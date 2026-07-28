import base64
import ctypes
import gzip
import json
import shutil
import sqlite3
import sys
import time
import secrets
from collections import defaultdict
from ctypes import wintypes
from pathlib import Path

from eprj2_reader import PUCHAR, _AuthInfo, _BCRYPT, _check, _decrypt_gcm, read_project


SCHEMATIC_UUID = "9e68e92bc81c2c02"
PCB_UUID = "a665c7e5bb4d30da"
M0_SCHEMATIC_ID = "e65b7223920e8ed6"
M0_PCB_ID = "b003c6432a60de36"

DELETE_DESIGNATORS = {"SW6", "BUZZER2", "Q1", "R2"}
LED_BLOCK_DESIGNATORS = {"LED1", "LED2", "LED3", "R1", "R3", "R4"}
REMOVED_NETS = {
    "KEY_UP",
    "KEY_DOWN",
    "KEY_LEFT",
    "KEY_RIGHT",
    "KEY_CENTER",
    "BUZZER",
    "$1N397",
    "$1N398",
}

SIGNAL_NETS = {
    69: "AIN1",       # PA14
    70: "AIN2",       # PA15
    67: "PWMA",       # PA16
    29: "BIN1",       # PA12
    30: "BIN2",       # PA13
    68: "PWMB",       # PA17
    50: "E1A",        # PA07
    7: "E1B",         # PA08
    25: "E2A",        # PB06
    27: "E2B",        # PB08
    15: "OLED_SCL",   # PB02
    17: "OLED_SDA",   # PB03
    5: "ICM_SCLK",    # PA01
    3: "ICM_SDA",     # PA00
    77: "ADC_read",   # PA27
    51: "AD0",        # PB00
    52: "AD1",        # PB01
    8: "AD2",         # PB04
}

GND_PINS = {1, 2, 21, 23, 37, 39, 41, 43, 54, 57, 59, 79, 80}
V5_PINS = {24, 40, 42, 58}
V3_PINS = {22, 38, 44, 60}


def _encrypt_gcm(key, nonce, plaintext):
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

        nonce_buffer = (ctypes.c_ubyte * len(nonce)).from_buffer_copy(nonce)
        input_buffer = (ctypes.c_ubyte * len(plaintext)).from_buffer_copy(plaintext)
        output_buffer = (ctypes.c_ubyte * len(plaintext))()
        tag_buffer = (ctypes.c_ubyte * 16)()
        auth = _AuthInfo()
        auth.cbSize = ctypes.sizeof(_AuthInfo)
        auth.dwInfoVersion = 1
        auth.pbNonce = nonce_buffer
        auth.cbNonce = len(nonce)
        auth.pbTag = tag_buffer
        auth.cbTag = len(tag_buffer)
        output_length = wintypes.ULONG()
        _check(
            _BCRYPT.BCryptEncrypt(
                key_handle,
                input_buffer,
                len(plaintext),
                ctypes.byref(auth),
                None,
                0,
                output_buffer,
                len(output_buffer),
                ctypes.byref(output_length),
                0,
            ),
            "BCryptEncrypt",
        )
        return bytes(output_buffer[: output_length.value]) + bytes(tag_buffer)
    finally:
        if key_handle:
            _BCRYPT.BCryptDestroyKey(key_handle)
        if algorithm:
            _BCRYPT.BCryptCloseAlgorithmProvider(algorithm, 0)


def _attributes(objects):
    result = defaultdict(dict)
    for (object_type, object_id), record in objects.items():
        if object_type != "ATTR":
            continue
        body = record["body"]
        result[body.get("parentId")][body.get("key")] = (object_id, body.get("value"))
    return result


def _component_ids_by_designator(objects):
    attrs = _attributes(objects)
    result = {}
    for (object_type, object_id), _record in objects.items():
        if object_type != "COMPONENT":
            continue
        value = attrs.get(object_id, {}).get("Designator", (None, None))[1]
        if value:
            result[value] = object_id
    return result


def _delete_component_records(objects, component_ids):
    deleted = set()
    pad_ids = set()
    for key, record in objects.items():
        object_type, object_id = key
        body = record["body"]
        parent = body.get("parentId")
        remove = object_type == "COMPONENT" and object_id in component_ids
        remove |= object_id.startswith(tuple(component_ids))
        remove |= parent in component_ids

        if object_type == "PAD_NET":
            parts = json.loads(object_id)
            if parts[1] in component_ids:
                remove = True
                pad_ids.add(parts[3])
        if remove:
            deleted.add(key)

    for key, record in objects.items():
        refs = record["body"].get("refs") or []
        if any(ref in pad_ids for ref in refs):
            deleted.add(key)
    return deleted


def _wire_groups_at_component_pins(documents, schematic, component_id):
    attrs = _attributes(schematic)
    component = schematic[("COMPONENT", component_id)]["body"]
    symbol_uuid = attrs[component_id]["Symbol"][1]
    symbol = documents[symbol_uuid]
    symbol_attrs = _attributes(symbol)
    angle = int(component.get("rotation") or 0) % 360

    pin_points = []
    for (object_type, object_id), record in symbol.items():
        if object_type != "PIN":
            continue
        x = record["body"]["x"]
        y = record["body"]["y"]
        if angle == 90:
            x, y = -y, x
        elif angle == 180:
            x, y = -x, -y
        elif angle == 270:
            x, y = y, -x
        number = int(symbol_attrs[object_id]["Pin Number"][1])
        pin_points.append((number, component["x"] + x, component["y"] + y))

    result = defaultdict(set)
    for (object_type, _object_id), record in schematic.items():
        if object_type != "LINE":
            continue
        body = record["body"]
        group = body.get("lineGroup")
        if not group:
            continue
        endpoints = [
            (body.get("startX"), body.get("startY")),
            (body.get("endX"), body.get("endY")),
        ]
        for number, x, y in pin_points:
            if any(
                px is not None
                and abs(px - x) < 0.01
                and abs(py - y) < 0.01
                for px, py in endpoints
            ):
                result[number].add(group)
    return result


def _wire_group_records(objects, groups):
    result = set()
    for key, record in objects.items():
        object_type, object_id = key
        body = record["body"]
        if object_type == "WIRE" and object_id in groups:
            result.add(key)
        elif body.get("parentId") in groups or body.get("lineGroup") in groups:
            result.add(key)
    return result


def _net_attr_for_group(objects, group):
    for (object_type, object_id), record in objects.items():
        body = record["body"]
        if (
            object_type == "ATTR"
            and body.get("parentId") == group
            and body.get("key") == "NET"
        ):
            return object_id
    raise ValueError(f"No NET attribute for wire group {group}")


def _pad_net_records(objects, component_id):
    result = {}
    for key, record in objects.items():
        if key[0] != "PAD_NET":
            continue
        parts = json.loads(key[1])
        if parts[1] == component_id:
            result[int(parts[2])] = (key, parts[3], record["body"].get("padNet", ""))
    return result


def _record(header, body=None, delete=False):
    left = json.dumps(header, ensure_ascii=False, separators=(",", ":"))
    if delete:
        return left + "|||"
    right = json.dumps(body, ensure_ascii=False, separators=(",", ":"))
    return left + "||" + right + "|"


def _merge_update(updates, key, body):
    updates.setdefault(key, {}).update(body)


def _translate_led_block(documents, schematic, dx, dy):
    designators = _component_ids_by_designator(schematic)
    component_ids = {designators[name] for name in LED_BLOCK_DESIGNATORS}
    component_groups = {
        component_id: set().union(
            *_wire_groups_at_component_pins(
                documents, schematic, component_id
            ).values()
        )
        for component_id in component_ids
    }
    groups = set().union(*component_groups.values())

    # Include the local 3V3 power symbols attached to the LED resistor wires.
    for (object_type, object_id), record in schematic.items():
        if object_type != "COMPONENT" or object_id in component_ids:
            continue
        if not record["body"].get("partId", "").startswith("pid"):
            continue
        attached = set().union(
            *_wire_groups_at_component_pins(
                documents, schematic, object_id
            ).values()
        )
        if attached & groups:
            component_ids.add(object_id)
            groups |= attached

    updates = {}
    for key, record in schematic.items():
        object_type, _object_id = key
        body = record["body"]
        if object_type == "COMPONENT" and key[1] in component_ids:
            _merge_update(
                updates,
                key,
                {"x": body["x"] + dx, "y": body["y"] + dy},
            )
        elif body.get("parentId") in component_ids:
            patch = {}
            if body.get("x") is not None:
                patch["x"] = body["x"] + dx
            if body.get("y") is not None:
                patch["y"] = body["y"] + dy
            if patch:
                _merge_update(updates, key, patch)
        elif object_type == "LINE" and body.get("lineGroup") in groups:
            _merge_update(
                updates,
                key,
                {
                    "startX": body["startX"] + dx,
                    "startY": body["startY"] + dy,
                    "endX": body["endX"] + dx,
                    "endY": body["endY"] + dy,
                },
            )
        elif body.get("parentId") in groups:
            patch = {}
            if body.get("x") is not None:
                patch["x"] = body["x"] + dx
            if body.get("y") is not None:
                patch["y"] = body["y"] + dy
            if patch:
                _merge_update(updates, key, patch)
    return updates


def _doc_head(metadata, ticket):
    return _record({"type": "DOCHEAD", "ticket": ticket}, metadata)


def build_patch(metadata, documents):
    schematic = documents[SCHEMATIC_UUID]
    pcb = documents[PCB_UUID]
    ticket = 180100
    patch = []

    def next_ticket():
        nonlocal ticket
        ticket += 1
        return ticket

    # Schematic: remove the direction key and complete buzzer driver chain.
    designators = _component_ids_by_designator(schematic)
    component_ids = {designators[name] for name in DELETE_DESIGNATORS}
    schematic_deletes = _delete_component_records(schematic, component_ids)
    for component_id in component_ids:
        groups = set().union(*_wire_groups_at_component_pins(
            documents, schematic, component_id
        ).values())
        schematic_deletes |= _wire_group_records(schematic, groups)

    # Schematic: retain only the requested M0 signal groups and power wiring.
    m0_groups = _wire_groups_at_component_pins(documents, schematic, M0_SCHEMATIC_ID)
    keep_pins = set(SIGNAL_NETS) | GND_PINS | V5_PINS | V3_PINS
    for pin, groups in m0_groups.items():
        if pin not in keep_pins:
            schematic_deletes |= _wire_group_records(schematic, groups)

    patch.append(_doc_head(metadata[SCHEMATIC_UUID], next_ticket()))
    for object_type, object_id in sorted(schematic_deletes):
        patch.append(
            _record(
                {"type": object_type, "ticket": next_ticket(), "id": object_id},
                delete=True,
            )
        )
    for pin, net in SIGNAL_NETS.items():
        groups = m0_groups.get(pin, set())
        if len(groups) != 1:
            raise ValueError(f"M0 schematic pin {pin} has {len(groups)} wire groups")
        attr_id = _net_attr_for_group(schematic, next(iter(groups)))
        patch.append(
            _record(
                {"type": "ATTR", "ticket": next_ticket(), "id": attr_id},
                {"value": net},
            )
        )

    # Reuse the former key area as a compact status-LED block.
    layout_deletes = {
        ("RECT", "8806667c308b9f74"),  # old buzzer frame
        ("TEXT", "06de4b38534fbc5d"),  # old buzzer title
        ("RECT", "297a4568b71b5025"),  # old LED frame
        ("TEXT", "ab44278585445205"),  # old LED title
        ("RECT", "ef85fd7d8f34daf8"),  # unused-pin frame
        ("TEXT", "b13d438ece1ed33b"),  # unused-pin title
    }
    for key, record in schematic.items():
        if key[0] != "TEXT":
            continue
        body = record["body"]
        if (
            body.get("x") is not None
            and body.get("y") is not None
            and 1090 <= body["x"] <= 1570
            and -330 <= body["y"] <= -240
        ):
            layout_deletes.add(key)
    for object_type, object_id in sorted(layout_deletes - schematic_deletes):
        patch.append(
            _record(
                {"type": object_type, "ticket": next_ticket(), "id": object_id},
                delete=True,
            )
        )

    layout_updates = _translate_led_block(documents, schematic, -280, 190)
    _merge_update(
        layout_updates,
        ("TEXT", "ee34e89588b3d3e5"),
        {"x": 600, "y": -595, "fontSize": 15, "value": "状态指示灯"},
    )
    for (object_type, object_id), body in sorted(layout_updates.items()):
        if (object_type, object_id) in schematic_deletes:
            continue
        patch.append(
            _record(
                {"type": object_type, "ticket": next_ticket(), "id": object_id},
                body,
            )
        )

    # PCB: remove matching components and their local/exclusive copper.
    pcb_designators = _component_ids_by_designator(pcb)
    pcb_component_ids = {pcb_designators[name] for name in DELETE_DESIGNATORS}
    pcb_deletes = _delete_component_records(pcb, pcb_component_ids)
    for key, record in pcb.items():
        body = record["body"]
        if body.get("netName") in REMOVED_NETS:
            pcb_deletes.add(key)
        if key[0] == "NET" and json.loads(key[1])[1] in REMOVED_NETS:
            pcb_deletes.add(key)

    pad_records = _pad_net_records(pcb, M0_PCB_ID)
    desired_pad_nets = {}
    for pin in range(1, 81):
        if pin in SIGNAL_NETS:
            desired_pad_nets[pin] = SIGNAL_NETS[pin]
        elif pin in GND_PINS:
            desired_pad_nets[pin] = "GND"
        elif pin in V5_PINS:
            desired_pad_nets[pin] = "+5V"
        elif pin in V3_PINS:
            desired_pad_nets[pin] = "3V3"
        else:
            desired_pad_nets[pin] = ""

    # Remove the copper segment and teardrop that directly touch every changed M0 pad.
    changed_pad_ids = {
        pad_records[pin][1]
        for pin, desired in desired_pad_nets.items()
        if pad_records[pin][2] != desired
    }
    for key, record in pcb.items():
        refs = record["body"].get("refs") or []
        if any(ref in changed_pad_ids for ref in refs):
            pcb_deletes.add(key)
            for ref in refs:
                for candidate in pcb:
                    if candidate[1] == ref and candidate[0] != "PAD_NET":
                        pcb_deletes.add(candidate)

    patch.append(_doc_head(metadata[PCB_UUID], next_ticket()))
    for object_type, object_id in sorted(pcb_deletes):
        patch.append(
            _record(
                {"type": object_type, "ticket": next_ticket(), "id": object_id},
                delete=True,
            )
        )
    for pin, desired in desired_pad_nets.items():
        key, _pad_id, actual = pad_records[pin]
        if actual == desired:
            continue
        patch.append(
            _record(
                {"type": "PAD_NET", "ticket": next_ticket(), "id": key[1]},
                {"padNet": desired},
            )
        )
    return "\n".join(patch), len(schematic_deletes), len(pcb_deletes)


def apply_revision(source, destination):
    metadata, documents = read_project(source)
    patch, schematic_delete_count, pcb_delete_count = build_patch(
        metadata, documents
    )
    shutil.copy2(source, destination)

    connection = sqlite3.connect(destination)
    connection.row_factory = sqlite3.Row
    try:
        branch = connection.execute(
            "SELECT branch_uuid FROM project_structures "
            "ORDER BY ticket DESC LIMIT 1"
        ).fetchone()[0]
        history_table = "project_history_" + branch
        history = connection.execute(
            f"SELECT * FROM {history_table} ORDER BY id DESC LIMIT 1"
        ).fetchone()
        data_uuid = history["uuid"]
        if history["num"]:
            data_uuid += f'-{history["num"]}'
        row = connection.execute(
            "SELECT dataStr FROM history_data WHERE uuid = ?", (data_uuid,)
        ).fetchone()
        packed = base64.b64decode(row[0])
        key = bytes.fromhex(history["key"])
        nonce = bytes.fromhex(history["uuid"])
        compressed = _decrypt_gcm(key, nonce, packed)
        original = gzip.decompress(compressed).decode("utf-8")
        updated = original.rstrip() + "\n" + patch
        repacked = _encrypt_gcm(key, nonce, gzip.compress(updated.encode("utf-8"), 1))
        encoded = base64.b64encode(repacked).decode("ascii")
        now = time.strftime("%Y-%m-%d %H:%M:%S")
        connection.execute(
            "UPDATE history_data SET dataStr = ?, updated_at = ? WHERE uuid = ?",
            (encoded, now, data_uuid),
        )
        old_project_uuid = connection.execute(
            "SELECT uuid FROM projects LIMIT 1"
        ).fetchone()[0]
        new_project_uuid = secrets.token_hex(32)
        for table_row in connection.execute(
            "SELECT name FROM sqlite_master WHERE type = 'table'"
        ).fetchall():
            table_name = table_row[0]
            columns = {
                row[1]
                for row in connection.execute(
                    f'PRAGMA table_info("{table_name}")'
                )
            }
            if "project_uuid" in columns:
                connection.execute(
                    f'UPDATE "{table_name}" SET project_uuid = ? '
                    "WHERE project_uuid = ?",
                    (new_project_uuid, old_project_uuid),
                )
        connection.execute(
            "UPDATE projects SET uuid = ?, name = ?, updated_at = ?",
            (new_project_uuid, "M0_1.2_clean", now),
        )
        connection.commit()
        check = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if check != "ok":
            raise RuntimeError(f"SQLite integrity check failed: {check}")
    finally:
        connection.close()
    return schematic_delete_count, pcb_delete_count


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: make_minimal_io_revision.py SOURCE DESTINATION")
    source = Path(sys.argv[1]).resolve()
    destination = Path(sys.argv[2]).resolve()
    if source == destination:
        raise SystemExit("destination must differ from source")
    sch_count, pcb_count = apply_revision(source, destination)
    print(f"created: {destination}")
    print(f"schematic records deleted: {sch_count}")
    print(f"pcb records deleted: {pcb_count}")
