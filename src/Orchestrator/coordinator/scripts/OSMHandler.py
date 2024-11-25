
# // Licensed under the Apache License, Version 2.0 (the "License");
# // you may not use this file except in compliance with the License.
# // You may obtain a copy of the License at
# //
# //     http://www.apache.org/licenses/LICENSE-2.0
# //
# // Unless required by applicable law or agreed to in writing, software
# // distributed under the License is distributed on an "AS IS" BASIS,
# // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# // See the License for the specific language governing permissions and
# // limitations under the License.

import osmium

class OSMHandler(osmium.SimpleHandler):
    def __init__(self):
        super(OSMHandler, self).__init__()
        self.relations = {}

    def way(self, w):
        way_coords = []
        for node_ref in w.nodes:
            node_id = node_ref.ref
            if node_id in self.data["nodes"]:
                node_info = self.data["nodes"][node_id]
                lat = node_info.get("lat")
                lon = node_info.get("lon")
                if lat is not None and lon is not None:
                    way_coords.append((lat, lon))
        self.ways[w.id] = way_coords

    def relation(self, r):
        relation_ways = []
        way = []
        # print(len(r.members))
        for member in r.members:
            if member.type == 'w' and member.ref in self.ways:
                way_coords = self.ways[member.ref]
                way.extend(way_coords)
            relation_ways.append(way)
            way = []
        self.relations[r.id] = relation_ways

    def node(self, n):
        self.data["nodes"][n.id] = {
            "lat": n.location.lat if n.location.valid() else None,
            "lon": n.location.lon if n.location.valid() else None,
            "tags": dict(n.tags),
        }

if __name__ == "__main__":
    osm_file = "/home/yxwork/SafeCLAI/safetylayerperception/safetylayerperception_ws/src/sc_perception_launch/scripts/lanelet2_map.osm"  # Replace with the path to your OSM XML file

    handler = OSMHandler()
    handler.ways = {}  # Initialize a dictionary to store ways
    handler.data = {"nodes": {}}

    handler.apply_file(osm_file)

    # Now, handler.relations is a dictionary where each key is the relation ID,
    # and the value is a list of coordinates for that relation.
    
    # To print all relations and their coordinates:
    print(handler.relations.items())
    for relation_id, relation_coords in handler.relations.items():
        print(f"Relation {relation_id}:")
        for way_coords in relation_coords:
                
                print(way_coords)


                