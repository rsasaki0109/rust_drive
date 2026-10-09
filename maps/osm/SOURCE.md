# Map data provenance and licensing

`german-road-extract.osm` is an unchanged, public OSMnx test-data file:

- Repository: https://github.com/gboeing/osmnx
- Pinned revision: `74e68ce2200b23c04f6ec2a864a6c24859bbf08d`
- Path: `tests/input_data/planet_10.068,48.135_10.071,48.137.osm`
- Raw source: https://raw.githubusercontent.com/gboeing/osmnx/74e68ce2200b23c04f6ec2a864a6c24859bbf08d/tests/input_data/planet_10.068,48.135_10.071,48.137.osm
- Extract timestamp: `2020-08-10T00:00:00Z`; generator: `osmconvert 0.8.11`.
- SHA-256: `280febb5b8b084cd4f29c138f5f65f8b219efdca7070bc7233a85ef972fab2a9`.

© OpenStreetMap contributors. This geographic data and its derived databases are provided under the [Open Data Commons Open Database License 1.0](https://opendatacommons.org/licenses/odbl/1-0/). [OSM copyright, attribution and license information](https://www.openstreetmap.org/copyright). Retain this attribution and the ODbL license notice when redistributing the data or derived databases. These map-data files are separately licensed from RustDriving's Apache-2.0 software. No OSMnx implementation is copied or used at runtime.

`german-road-extract.json` is a mechanical representation of those same XML node coordinates, way node references/tags and relation members/tags in Overpass JSON shape. It adds the above attribution, omits XML editing metadata and preserves geographic values. Its SHA-256 is `77189ddb7f66915817d11b83765851709b409713487e5d2b9d8f357f665680c6`. The original XML remains available for independent comparison. The normalized JSON, `german-road-network.json`, its import report and all `german-road-*-scenario.json` variants are derived OSM data under ODbL 1.0. Separate simulation ground/obstacles and body dimensions are authored calibration, not geographic measurements. The extract is historical and cropped; current physical road accuracy is not established.

`authored-junction.json` is an original RustDriving test fixture, under Apache-2.0. It uses the OSM node/way vocabulary to test shared junctions, directed routing and closures, and contains no real OSM data. It must not be represented as an external geographic extract.

The branch fixture `scenarios/local-corners-german-branch.json` also contains the same derived OSM network and is separately licensed under ODbL 1.0. Its planner option and simulator calibration are RustDriving-authored. The synthetic `scenarios/local-corners-authored-detour.json` contains no OSM data and remains Apache-2.0.

Regenerate the JSON representation with Python's standard library:

```python
import json
import xml.etree.ElementTree as ET

root = ET.parse("maps/osm/german-road-extract.osm").getroot()
elements = []
for item in root:
    if item.tag not in ("node", "way", "relation"):
        continue
    value = {"type": item.tag, "id": int(item.attrib["id"])}
    if item.tag == "node":
        value.update(lat=float(item.attrib["lat"]), lon=float(item.attrib["lon"]))
    elif item.tag == "way":
        value["nodes"] = [int(n.attrib["ref"]) for n in item.findall("nd")]
    else:
        value["members"] = [dict(n.attrib) for n in item.findall("member")]
    tags = {t.attrib["k"]: t.attrib["v"] for t in item.findall("tag")}
    if tags:
        value["tags"] = tags
    elements.append(value)
document = {
    "version": 0.6,
    "generator": "RustDriving mechanical XML-to-Overpass-JSON representation",
    "osm3s": {
        "timestamp_osm_base": root.attrib["timestamp"],
        "copyright": "© OpenStreetMap contributors; ODbL 1.0 https://www.openstreetmap.org/copyright",
    },
    "elements": elements,
}
with open("maps/osm/german-road-extract.json", "w") as output:
    json.dump(document, output, indent=2)
```
