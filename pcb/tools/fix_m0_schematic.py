import argparse
import base64
import gzip
import hashlib
import json
import os
import shutil
import sqlite3
import time
from collections import defaultdict
from pathlib import Path

from eprj2_reader import _decrypt_gcm, read_project
from make_minimal_io_revision import _encrypt_gcm


SCHEMATIC_UUID = "9e68e92bc81c2c02"
PCB_UUID = "a665c7e5bb4d30da"
M0_SCHEMATIC_ID = "e65b7223920e8ed6"

# Pin numbers are the 80-pin LCKFB-TMX-MSPM0G3507 module header numbers.
SIGNAL_NETS = {
    3: "ICM_SDA",       # PA0, MPU6050 SDA
    5: "ICM_SCLK",      # PA1, MPU6050 SCL
    7: "E1B",           # PA8, left encoder B
    8: "AD2",           # PB4, analog tracking address 2
    15: "OLED_SCL",     # PB2
    17: "OLED_SDA",     # PB3
    25: "E2A",          # PB6, right encoder A
    27: "E2B",          # PB8, right encoder B
    29: "BIN1",         # PA12
    30: "BIN2",         # PA13
    50: "E1A",          # PA7, left encoder A
    51: "AD0",          # PB0, analog tracking address 0
    52: "AD1",          # PB1, analog tracking address 1
    67: "PWMA",         # PA16
    68: "PWMB",         # PA17
    69: "AIN1",         # PA14
    70: "AIN2",         # PA15
    77: "ADC_read",     # PA27, analog tracking output
}

GND_PINS = {1, 2, 21, 23, 37, 39, 41, 43, 54, 57, 59, 79, 80}
V5_PINS = {24, 40, 42, 58}

DELETE_DESIGNATORS = {
    "CN5",
    "CN6",
    "CN7",
    "CN8",
    "SW6",
    "BUZZER2",
    "Q1",
    "R2",
    "H6",
}

RENAME_NETS = {
    "M-SCL": "ICM_SCLK",
    "M-SDA": "ICM_SDA",
    "O-SCL": "OLED_SCL",
    "O-SDA": "OLED_SDA",
}

# Decorative blocks that belong to removed interfaces or stale spare-pin lists.
DELETE_LAYOUT_IDS = {
    ("RECT", "ee3061d8ed46978c"),  # UART block
    ("TEXT", "5d67c97a3c6ad0ec"),
    ("RECT", "e5e621b18c0dbbe9"),  # key block
    ("TEXT", "ee34e89588b3d3e5"),
    ("RECT", "8806667c308b9f74"),  # buzzer block
    ("TEXT", "06de4b38534fbc5d"),
    ("RECT", "ef85fd7d8f34daf8"),  # stale spare-pin block
    ("TEXT", "b13d438ece1ed33b"),
}


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _attributes(objects):
    result = defaultdict(dict)
    for (object_type, object_id), record in objects.items():
        if object_type != "ATTR":
            continue
        body = record["body"]
        result[body.get("parentId")][body.get("key")] = (
            object_id,
            body.get("value"),
        )
    return result


def _designators(objects):
    attrs = _attributes(objects)
    result = {}
    for (object_type, object_id), _record in objects.items():
        if object_type != "COMPONENT":
            continue
        value = attrs.get(object_id, {}).get("Designator", (None, None))[1]
        if value:
            result[value] = object_id
    return result


def _component_pin_groups(documents, schematic, component_id):
    attrs = _attributes(schematic)
    component = schematic[("COMPONENT", component_id)]["body"]
    symbol_uuid = attrs[component_id]["Symbol"][1]
    symbol = documents.get(symbol_uuid)
    if symbol is None:
        # Some built-in one-pin power symbols are referenced from the cloud
        # library and are not embedded in the project. Their component origin
        # is the electrical connection point, which is enough for cleanup.
        result = defaultdict(set)
        x = component.get("x")
        y = component.get("y")
        for (object_type, _object_id), record in schematic.items():
            if object_type != "LINE":
                continue
            body = record["body"]
            group = body.get("lineGroup")
            endpoints = (
                (body.get("startX"), body.get("startY")),
                (body.get("endX"), body.get("endY")),
            )
            if any(
                px is not None
                and abs(px - x) < 0.01
                and abs(py - y) < 0.01
                for px, py in endpoints
            ):
                result[1].add(group)
        return result
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
        endpoints = (
            (body.get("startX"), body.get("startY")),
            (body.get("endX"), body.get("endY")),
        )
        for number, x, y in pin_points:
            if any(
                px is not None
                and abs(px - x) < 0.01
                and abs(py - y) < 0.01
                for px, py in endpoints
            ):
                result[number].add(group)
    return result


def _groups_for_components(documents, schematic, component_ids):
    groups = set()
    for component_id in component_ids:
        pin_groups = _component_pin_groups(documents, schematic, component_id)
        for values in pin_groups.values():
            groups.update(values)
    return groups


def _records_for_groups(objects, groups):
    result = set()
    for key, record in objects.items():
        object_type, object_id = key
        body = record["body"]
        if object_type == "WIRE" and object_id in groups:
            result.add(key)
        elif body.get("parentId") in groups or body.get("lineGroup") in groups:
            result.add(key)
    return result


def _records_for_components(objects, component_ids):
    result = set()
    prefixes = tuple(component_ids)
    for key, record in objects.items():
        object_type, object_id = key
        body = record["body"]
        if object_type == "COMPONENT" and object_id in component_ids:
            result.add(key)
        elif prefixes and object_id.startswith(prefixes):
            result.add(key)
        elif body.get("parentId") in component_ids:
            result.add(key)
    return result


def _auxiliary_components_on_groups(documents, schematic, groups, protected_ids):
    designator_ids = set(_designators(schematic).values())
    result = set()
    for (object_type, object_id), record in schematic.items():
        if object_type != "COMPONENT" or object_id in protected_ids:
            continue
        # Only remove net ports and local power symbols. Named functional
        # components are kept even when one of their pins becomes unused.
        if object_id in designator_ids:
            continue
        pin_groups = _component_pin_groups(documents, schematic, object_id)
        attached = set().union(*pin_groups.values()) if pin_groups else set()
        if attached & groups:
            result.add(object_id)
    return result


def _record(header, body=None, delete=False):
    left = json.dumps(header, ensure_ascii=False, separators=(",", ":"))
    if delete:
        return left + "|||"
    right = json.dumps(body, ensure_ascii=False, separators=(",", ":"))
    return left + "||" + right + "|"


def _doc_head(metadata, ticket):
    return _record({"type": "DOCHEAD", "ticket": ticket}, metadata)


def _pcb_digest(documents):
    items = []
    for key, value in documents[PCB_UUID].items():
        items.append((key, value["header"], value["body"]))
    packed = json.dumps(
        sorted(items), ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(packed).hexdigest()


def build_patch(metadata, documents):
    schematic = documents[SCHEMATIC_UUID]
    designators = _designators(schematic)
    missing = sorted(DELETE_DESIGNATORS - set(designators))
    if missing:
        raise ValueError("Missing expected schematic components: " + ", ".join(missing))

    ticket = max(
        int(record["header"].get("ticket") or 0)
        for record in schematic.values()
    ) + 100

    def next_ticket():
        nonlocal ticket
        ticket += 1
        return ticket

    delete_component_ids = {designators[name] for name in DELETE_DESIGNATORS}
    delete_groups = _groups_for_components(
        documents, schematic, delete_component_ids
    )

    m0_groups = _component_pin_groups(
        documents, schematic, M0_SCHEMATIC_ID
    )
    keep_pins = set(SIGNAL_NETS) | GND_PINS | V5_PINS
    for pin, groups in m0_groups.items():
        if pin not in keep_pins:
            delete_groups.update(groups)

    # Remove net-port/power-symbol graphics that would otherwise be left
    # floating after deleting a wire group.
    protected_ids = {M0_SCHEMATIC_ID} | set(designators.values())
    auxiliary_ids = _auxiliary_components_on_groups(
        documents, schematic, delete_groups, protected_ids
    )
    delete_component_ids.update(auxiliary_ids)

    delete_records = _records_for_groups(schematic, delete_groups)
    delete_records.update(
        _records_for_components(schematic, delete_component_ids)
    )
    delete_records.update(DELETE_LAYOUT_IDS)

    # Remove all text entries inside the stale spare-pin frame.
    for key, record in schematic.items():
        if key[0] != "TEXT":
            continue
        body = record["body"]
        x = body.get("x")
        y = body.get("y")
        if x is not None and y is not None and 1090 <= x <= 1570 and -330 <= y <= -240:
            delete_records.add(key)

    updates = {}

    def update(key, body):
        if key not in delete_records:
            updates.setdefault(key, {}).update(body)

    def set_group_net(groups, net):
        if len(groups) != 1:
            raise ValueError(f"Expected one wire group for {net}, got {len(groups)}")
        group = next(iter(groups))
        attrs = []
        for key, record in schematic.items():
            body = record["body"]
            if (
                key[0] == "ATTR"
                and body.get("parentId") == group
                and body.get("key") == "NET"
            ):
                attrs.append(key)
        if not attrs:
            raise ValueError(f"Wire group {group} has no NET attribute")
        m0 = schematic[("COMPONENT", M0_SCHEMATIC_ID)]["body"]
        attrs.sort(
            key=lambda key: (
                (schematic[key]["body"].get("x") or 0) - m0["x"]
            ) ** 2
            + (
                (schematic[key]["body"].get("y") or 0) - m0["y"]
            ) ** 2
        )
        update(attrs[0], {"value": net})
        for duplicate in attrs[1:]:
            delete_records.add(duplicate)

    for pin, net in SIGNAL_NETS.items():
        set_group_net(m0_groups.get(pin, set()), net)
    for pin in GND_PINS:
        set_group_net(m0_groups.get(pin, set()), "GND")
    for pin in V5_PINS:
        set_group_net(m0_groups.get(pin, set()), "+5V")

    for key, record in schematic.items():
        body = record["body"]
        if key[0] == "ATTR" and body.get("key") == "NET":
            replacement = RENAME_NETS.get(body.get("value"))
            if replacement:
                update(key, {"value": replacement})

    # Make the two newly added modules easier to identify in the schematic.
    update(("TEXT", "5b061ef683c35ea5"), {"value": "MPU6050陀螺仪"})

    patch = [_doc_head(metadata[SCHEMATIC_UUID], next_ticket())]
    for object_type, object_id in sorted(delete_records):
        if (object_type, object_id) not in schematic:
            continue
        patch.append(
            _record(
                {"type": object_type, "ticket": next_ticket(), "id": object_id},
                delete=True,
            )
        )
    for (object_type, object_id), body in sorted(updates.items()):
        patch.append(
            _record(
                {"type": object_type, "ticket": next_ticket(), "id": object_id},
                body,
            )
        )
    return "\n".join(patch), len(delete_records), len(updates)


def _append_patch(project_path, patch):
    connection = sqlite3.connect(project_path)
    connection.row_factory = sqlite3.Row
    try:
        branch = connection.execute(
            "SELECT branch_uuid FROM project_structures ORDER BY ticket DESC LIMIT 1"
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
        repacked = _encrypt_gcm(
            key, nonce, gzip.compress(updated.encode("utf-8"), 1)
        )
        encoded = base64.b64encode(repacked).decode("ascii")
        now = time.strftime("%Y-%m-%d %H:%M:%S")
        connection.execute(
            "UPDATE history_data SET dataStr = ?, updated_at = ? WHERE uuid = ?",
            (encoded, now, data_uuid),
        )
        connection.execute(
            "UPDATE projects SET updated_at = ?", (now,)
        )
        connection.commit()
        check = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if check != "ok":
            raise RuntimeError("SQLite integrity check failed: " + check)
    finally:
        connection.close()


def verify_project(project_path, original_pcb_digest):
    _metadata, documents = read_project(project_path)
    if _pcb_digest(documents) != original_pcb_digest:
        raise RuntimeError("PCB document changed unexpectedly")

    schematic = documents[SCHEMATIC_UUID]
    designators = _designators(schematic)
    remaining = sorted(DELETE_DESIGNATORS & set(designators))
    if remaining:
        raise RuntimeError("Removed components still present: " + ", ".join(remaining))

    groups = _component_pin_groups(documents, schematic, M0_SCHEMATIC_ID)
    expected = dict(SIGNAL_NETS)
    expected.update({pin: "GND" for pin in GND_PINS})
    expected.update({pin: "+5V" for pin in V5_PINS})
    for pin in range(1, 81):
        pin_groups = groups.get(pin, set())
        if pin in expected:
            if len(pin_groups) != 1:
                raise RuntimeError(f"M0 pin {pin} has {len(pin_groups)} groups")
            group = next(iter(pin_groups))
            values = {
                record["body"].get("value")
                for key, record in schematic.items()
                if key[0] == "ATTR"
                and record["body"].get("parentId") == group
                and record["body"].get("key") == "NET"
            }
            if values != {expected[pin]}:
                raise RuntimeError(
                    f"M0 pin {pin} expected {expected[pin]}, got {sorted(values)}"
                )
        elif pin_groups:
            raise RuntimeError(f"Unused M0 pin {pin} is still wired")

    net_values = [
        record["body"].get("value")
        for key, record in schematic.items()
        if key[0] == "ATTR" and record["body"].get("key") == "NET"
    ]
    for old in RENAME_NETS:
        if old in net_values:
            raise RuntimeError("Stale net remains: " + old)
    for net in SIGNAL_NETS.values():
        if net_values.count(net) < 2:
            raise RuntimeError(f"Net {net} does not reach a peripheral")

    connection = sqlite3.connect(f"file:{Path(project_path).resolve()}?mode=ro", uri=True)
    try:
        check = connection.execute("PRAGMA integrity_check").fetchone()[0]
        if check != "ok":
            raise RuntimeError("Final SQLite integrity check failed: " + check)
    finally:
        connection.close()


def apply_in_place(project_path):
    project_path = Path(project_path).resolve()
    original_hash = _sha256(project_path)
    metadata, documents = read_project(project_path)
    original_pcb_digest = _pcb_digest(documents)
    patch, delete_count, update_count = build_patch(metadata, documents)

    temporary = project_path.with_suffix(project_path.suffix + ".tmp")
    if temporary.exists():
        temporary.unlink()
    shutil.copy2(project_path, temporary)
    try:
        _append_patch(temporary, patch)
        verify_project(temporary, original_pcb_digest)

        if _sha256(project_path) != original_hash:
            raise RuntimeError("Source project changed while the patch was being tested")

        backup_dir = project_path.parent / "backups"
        backup_dir.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        backup = backup_dir / f"{project_path.name}.{stamp}.bak"
        shutil.copy2(project_path, backup)
        os.replace(temporary, project_path)
        verify_project(project_path, original_pcb_digest)
        return backup, delete_count, update_count
    finally:
        if temporary.exists():
            temporary.unlink()


def main():
    parser = argparse.ArgumentParser(
        description="Fix only the M0_1.0 schematic; leave the PCB document unchanged."
    )
    parser.add_argument("project", type=Path)
    args = parser.parse_args()
    backup, deletes, updates = apply_in_place(args.project)
    print("updated:", args.project.resolve())
    print("backup:", backup)
    print("schematic records deleted:", deletes)
    print("schematic records updated:", updates)


if __name__ == "__main__":
    main()
