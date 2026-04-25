import csv
import json
import os

def load_mapping(csv_path):
    mapping = {}
    json_path = os.path.splitext(csv_path)[0] + ".json"

    def clean(value):
        if value is None:
            return None
        value = value.strip()
        if not value or value.upper() in {"N/A", "NA", "NONE"}:
            return None
        return value

    def normalize_upper(value):
        value = clean(value)
        return value.upper() if value else None

    def validate_ifc_entity(entity, label):
        if not entity:
            print(f"Skipping label '{label}' due to invalid IFC Entity")
            return None
        if not entity.startswith("Ifc"):
            print(f"Warning: Invalid IFC Entity '{entity}' in label '{label}'")
            print(f"Skipping label '{label}' due to invalid IFC Entity")
            return None
        return entity

    def validate_predefined(value, label):
        if not value:
            return None
        value = value.upper()
        if "=" in value or "/" in value:
            print(f"Warning: Invalid PredefinedType '{value}' in label '{label}'")
            return None
        return value

    def validate_pset_key(key, label):
        if not key.startswith("Pset_") or "." not in key:
            print(f"Warning: Invalid Pset column '{key}' in label '{label}'")
            return None, None
        return key.split(".", 1)

    # --- JSON LOAD ---
    if os.path.exists(json_path):
        try:
            print("Loading mapping from JSON...")
            with open(json_path, "r", encoding="utf-8") as jf:
                return json.load(jf)
        except Exception as e:
            print(f"Warning: Failed to load JSON: {e}. Falling back to CSV...")

    # --- CSV PARSE ---
    print("Parsing CSV and saving JSON...")

    try:
        with open(csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)

            for row in reader:
                label_raw = row.get("Label")
                label = clean(label_raw)
                if not label:
                    continue

                key = label.lower()
                if key in mapping:
                    print(f"Warning: Duplicate label '{label}' found. Overwriting.")

                ifc_entity = validate_ifc_entity(clean(row.get("IFC Entity")), label)
                if ifc_entity is None:
                    continue

                name = normalize_upper(row.get("Name Attribute"))
                predefined = validate_predefined(normalize_upper(row.get("PredefinedType")), label)
                object_type = normalize_upper(row.get("ObjectType"))

                entry = {
                    "IfcEntity": ifc_entity,
                    "Name": name,
                    "PredefinedType": predefined,
                    "ObjectType": object_type,
                    "Psets": {}
                }

                for col, val in row.items():
                    if col.startswith("Pset_"):
                        pset_name, prop_name = validate_pset_key(col, label)
                        if not pset_name:
                            continue

                        pset_val = normalize_upper(val)
                        if pset_val is None:
                            continue

                        if pset_name not in entry["Psets"]:
                            entry["Psets"][pset_name] = {}

                        entry["Psets"][pset_name][prop_name] = pset_val

                mapping[key] = entry

    except FileNotFoundError:
        print(f"Warning: {csv_path} not found.")
        return {}

    # --- JSON SAVE ---
    try:
        with open(json_path, "w", encoding="utf-8") as jf:
            json.dump(mapping, jf, indent=2)
        print("Mapping saved to JSON")
    except Exception as e:
        print(f"Warning: Failed to save JSON: {e}")

    return mapping

if __name__ == "__main__":
    load_mapping("mapping.csv")
