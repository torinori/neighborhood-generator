import sys
import json
from graph import TGraph, read_graph_from_json
from planegeometry.structures.segments import Segment, Point
from linesegmentintersections import bentley_ottman

if __name__ == "__main__":
    
    # Read input file from args
    file_name = sys.argv[1]

    graph: TGraph = read_graph_from_json(f"../data_samples/osm_export/{file_name}.json")

    print(graph)

    segment_list = list()

    MOVE = 0.000001

    for src_index in range(len(graph.nodes)):
        src = graph.nodes[src_index]
        for dst_index in graph.edges[src_index]:
            if src_index >= dst_index:
                continue
                
            dst = graph.nodes[dst_index]
            
            # print(src.lat, src.lng, dst.lat, dst.lng)

            p1 = Point(src.lat, src.lng)
            p2 = Point(dst.lat, dst.lng)
            seg = Segment(p1, p2)
            
            # Need to move endpoints to center by a small amount.
            center = seg.center()
            dir1 = p1 - center
            new_p1 = p1 - dir1 * (1 / dir1.__abs__()) * MOVE

            dir2 = p2 - center
            new_p2 = p2 - dir2 * (1 / dir2.__abs__()) * MOVE

            segment_list.append([[new_p1.x, new_p1.y], [new_p2.x, new_p2.y]])

    li = bentley_ottman(segment_list)

    for intersection in li:
        print(intersection.x, intersection.y)
        
    segment_list = list()

    MOVE = 0.000001

    edges_to_remove = []

    for inter in li:
        inter_point = Point(inter.x, inter.y)
        for src_index in range(len(graph.nodes)):
            src = graph.nodes[src_index]
            for dst_index in graph.edges[src_index]:
                if src_index >= dst_index:
                    continue
                    
                dst = graph.nodes[dst_index]
                
                # print(src.lat, src.lng, dst.lat, dst.lng)

                p1 = Point(src.lat, src.lng)
                p2 = Point(dst.lat, dst.lng)
                seg = Segment(p1, p2)
                
                if min(p1.x, p2.x) <= inter_point.x and inter_point.x <= max(p1.x, p2.x) and (abs(seg.calculate_y(inter_point.x) - inter_point.y) < 0.00001):
                    # print(seg, inter_point)
                    edges_to_remove.append((src_index, dst_index))

    for edge in edges_to_remove:
        src_index, dst_index = edge
        if dst_index in graph.edges[src_index]:
            graph.edges[src_index].remove(dst_index)
        if src_index in graph.edges[dst_index]:
            graph.edges[dst_index].remove(src_index)
            
    # Format to graph to json format
    data = {
        "elements": []
    }

    for node in graph.nodes:
        data['elements'].append({
            "type": "node",
            "id": node.id,
            "lat": node.lat,
            "lon": node.lng
        })

    for src_index in range(len(graph.nodes)):
        for dst_index in graph.edges[src_index]:
            src = graph.nodes[src_index]
            dst = graph.nodes[dst_index]
            
            if src.id >= dst.id:
                continue
            
            data['elements'].append({
                "type": "way",
                "nodes": [src.id, dst.id]
            })

    with open(f'../data_samples/prepared/{file_name}.json', 'w') as file:
        json.dump(data, file)