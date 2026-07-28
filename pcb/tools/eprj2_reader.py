import base64
import ctypes
import gzip
import json
import sqlite3
from ctypes import wintypes
from pathlib import Path


PUCHAR = ctypes.POINTER(ctypes.c_ubyte)
_BCRYPT = ctypes.WinDLL("bcrypt.dll")


class _AuthInfo(ctypes.Structure):
    _fields_ = [
        ("cbSize", wintypes.ULONG),
        ("dwInfoVersion", wintypes.ULONG),
        ("pbNonce", PUCHAR),
        ("cbNonce", wintypes.ULONG),
        ("pbAuthData", PUCHAR),
        ("cbAuthData", wintypes.ULONG),
        ("pbTag", PUCHAR),
        ("cbTag", wintypes.ULONG),
        ("pbMacContext", PUCHAR),
        ("cbMacContext", wintypes.ULONG),
        ("cbAAD", wintypes.ULONG),
        ("cbData", ctypes.c_ulonglong),
        ("dwFlags", wintypes.ULONG),
    ]


def _check(status, operation):
    if status:
        raise OSError(f"{operation} failed: 0x{status & 0xFFFFFFFF:08X}")


def _decrypt_gcm(key, nonce, packed):
    algorithm = wintypes.HANDLE()
    key_handle = wintypes.HANDLE()
    _check(
        _BCRYPT.BCryptOpenAlgorithmProvider(
            ctypes.byref(algorithm), "AES", None, 0
        ),
        "BCryptOpenAlgorithmProvider",
    )
    try:
        mode = ctypes.create_unicode_buffer("ChainingModeGCM")
        _check(
            _BCRYPT.BCryptSetProperty(
                algorithm,
                "ChainingMode",
                ctypes.cast(mode, PUCHAR),
                ctypes.sizeof(mode),
                0,
            ),
            "BCryptSetProperty",
        )

        object_length = wintypes.ULONG()
        result_length = wintypes.ULONG()
        _check(
            _BCRYPT.BCryptGetProperty(
                algorithm,
                "ObjectLength",
                ctypes.cast(ctypes.byref(object_length), PUCHAR),
                ctypes.sizeof(object_length),
                ctypes.byref(result_length),
                0,
            ),
            "BCryptGetProperty",
        )

        key_object = (ctypes.c_ubyte * object_length.value)()
        key_buffer = (ctypes.c_ubyte * len(key)).from_buffer_copy(key)
        _check(
            _BCRYPT.BCryptGenerateSymmetricKey(
                algorithm,
                ctypes.byref(key_handle),
                key_object,
                object_length.value,
                key_buffer,
                len(key),
                0,
            ),
            "BCryptGenerateSymmetricKey",
        )

        ciphertext, tag = packed[:-16], packed[-16:]
        nonce_buffer = (ctypes.c_ubyte * len(nonce)).from_buffer_copy(nonce)
        tag_buffer = (ctypes.c_ubyte * len(tag)).from_buffer_copy(tag)
        ciphertext_buffer = (ctypes.c_ubyte * len(ciphertext)).from_buffer_copy(
            ciphertext
        )
        plaintext_buffer = (ctypes.c_ubyte * len(ciphertext))()

        auth = _AuthInfo()
        auth.cbSize = ctypes.sizeof(_AuthInfo)
        auth.dwInfoVersion = 1
        auth.pbNonce = nonce_buffer
        auth.cbNonce = len(nonce)
        auth.pbTag = tag_buffer
        auth.cbTag = len(tag)

        plaintext_length = wintypes.ULONG()
        _check(
            _BCRYPT.BCryptDecrypt(
                key_handle,
                ciphertext_buffer,
                len(ciphertext),
                ctypes.byref(auth),
                None,
                0,
                plaintext_buffer,
                len(plaintext_buffer),
                ctypes.byref(plaintext_length),
                0,
            ),
            "BCryptDecrypt",
        )
        return bytes(plaintext_buffer[: plaintext_length.value])
    finally:
        if key_handle:
            _BCRYPT.BCryptDestroyKey(key_handle)
        if algorithm:
            _BCRYPT.BCryptCloseAlgorithmProvider(algorithm, 0)


def _records(text):
    decoder = json.JSONDecoder()
    position = 0
    while True:
        while position < len(text) and text[position].isspace():
            position += 1
        if position >= len(text):
            return

        header, position = decoder.raw_decode(text, position)
        if text[position : position + 2] != "||":
            raise ValueError(f"Invalid record at offset {position}")
        position += 2

        if position >= len(text):
            body = None
        elif text[position] == "|":
            body = None
            position += 1
        else:
            body, position = decoder.raw_decode(text, position)
            if position < len(text) and text[position] == "|":
                position += 1
        yield header, body


def read_project(project_path):
    path = Path(project_path).resolve()
    connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    connection.row_factory = sqlite3.Row
    try:
        branch = connection.execute(
            "SELECT branch_uuid FROM project_structures "
            "ORDER BY ticket DESC LIMIT 1"
        ).fetchone()[0]
        history_table = "project_history_" + branch
        documents = {}
        metadata = {}

        for history in connection.execute(
            f"SELECT * FROM {history_table} ORDER BY id"
        ):
            data_uuid = history["uuid"]
            if history["num"]:
                data_uuid += f'-{history["num"]}'
            row = connection.execute(
                "SELECT dataStr FROM history_data WHERE uuid = ?", (data_uuid,)
            ).fetchone()
            if not row:
                continue

            packed = base64.b64decode(row[0])
            compressed = _decrypt_gcm(
                bytes.fromhex(history["key"]),
                bytes.fromhex(history["uuid"]),
                packed,
            )
            text = gzip.decompress(compressed).decode("utf-8")
            current_document = None

            for header, body in _records(text):
                object_type = header.get("type")
                if object_type == "DOCHEAD":
                    current_document = body.get("uuid") if body else None
                    if current_document:
                        metadata.setdefault(current_document, {}).update(body)
                        documents.setdefault(current_document, {})
                    continue
                if (
                    object_type == "EDIT_HEAD"
                    or not current_document
                    or "id" not in header
                ):
                    continue

                key = (object_type, str(header["id"]))
                if body is None:
                    documents[current_document].pop(key, None)
                elif key in documents[current_document]:
                    documents[current_document][key]["header"].update(header)
                    documents[current_document][key]["body"].update(body)
                else:
                    documents[current_document][key] = {
                        "header": dict(header),
                        "body": dict(body),
                    }
        return metadata, documents
    finally:
        connection.close()


def component_pad_nets(objects, component_id):
    result = {}
    prefix = f'["PAD_NET","{component_id}",'
    for (object_type, object_id), record in objects.items():
        if object_type != "PAD_NET" or not object_id.startswith(prefix):
            continue
        pad_number = json.loads(object_id)[2]
        result[int(pad_number)] = record["body"].get("padNet", "")
    return dict(sorted(result.items()))
