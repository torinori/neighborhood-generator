import json

class TNode:
    def __init__(self, id, lat, lng):
        self.id = id
        self.lat = lat
        self.lng = lng

class TGraph:
    node_id_to_index = {}
    nodes = []
    edges: list[set] = []
    edges_list: list[list] = []

    def __init__(self) -> None:
        self.node_id_to_index = {}
        self.nodes = []
        self.edges = []
        self.edges_list = []

    def add_node(self, node: TNode, ok_if_exists=True):
        if node.id in self.node_id_to_index:
            if not ok_if_exists:
                raise Exception("Node already exists")
            return
        index = len(self.nodes)
        self.node_id_to_index[node.id] = index
        self.nodes.append(node)
        self.edges.append(set())

    def add_edge(self, src_id, dst_id, ok_if_not_exists=True):
        if src_id not in self.node_id_to_index:
            if not ok_if_not_exists:
                raise Exception("Source node does not exist")
            return
        if dst_id not in self.node_id_to_index:
            if not ok_if_not_exists:
                raise Exception("Destination node does not exist")
            return
        
        src_index = self.node_id_to_index[src_id]
        dst_index = self.node_id_to_index[dst_id]
        self.edges[src_index].add(dst_index)
        self.edges[dst_index].add(src_index)
        self.edges_list.append([src_index, dst_index])

    def __str__(self) -> str:
        return f"{len(self.nodes)} nodes, {len(self.edges_list)} edges"

def read_graph_from_json(file_path: str) -> TGraph:
    with open(file_path, 'r') as file:
        data = json.load(file)
        graph = TGraph()
        for element in data['elements']:
            if element['type'] == 'node':
                graph.add_node(TNode(element['id'], element['lat'], element['lon']))
        
        for element in data['elements']:
            if element['type'] == 'way':
                for i in range(len(element['nodes']) - 1):
                    src = element['nodes'][i]
                    dst = element['nodes'][i + 1]
                    graph.add_edge(src, dst)

        return graph