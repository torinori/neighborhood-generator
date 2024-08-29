#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "../include/rapidjson/document.h"
#include "../include/rapidjson/error/en.h"
#include "../include/rapidjson/stringbuffer.h"
#include "../include/rapidjson/writer.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/ref.hpp>

#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <boost/graph/planar_face_traversal.hpp>

#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/segment.hpp>

typedef uint64_t Id;
typedef size_t Index;

using namespace boost;
typedef geometry::model::point<double, 2, geometry::cs::cartesian> point_t;
typedef geometry::model::segment<point_t> segment_t;
typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_index_t, int>,
                       property<edge_index_t, int>>
    PlanarGraph;

struct TNode {
  Id id;
  Index index;
  double lat;
  double lng;

  TNode(Id id, Index index, double lat, double lng)
      : id(id), index(index), lat(lat), lng(lng) {}
};

struct TGraph {
  std::vector<TNode> nodes;
  std::vector<std::unordered_set<Index>> edges;
  std::vector<Id> node_ids;
  std::unordered_map<Id, Index> node_id_to_index;
  size_t _number_of_edges = 0;

  void add_node(Id id, double lat, double lng) {
    if (node_id_to_index.count(id)) {
      throw std::runtime_error("Node already exists");
    }

    TNode node(id, nodes.size(), lat, lng);
    nodes.push_back(node);
    node_ids.push_back(id);
    node_id_to_index[node.id] = node.index;
    edges.emplace_back();
  }

  bool add_edge(Id node_a, Id node_b) {
    if (!node_id_to_index.count(node_a) or !node_id_to_index.count(node_b)) {
      return false;
    }

    Index index_a = node_id_to_index.at(node_a);
    Index index_b = node_id_to_index.at(node_b);

    bool added = edges[index_a].insert(index_b).second;
    assert(edges[index_b].insert(index_a).second == added);

    _number_of_edges += added;
    return added;
  }

  bool remove_edge(Id node_a, Id node_b) {
    if (!node_id_to_index.count(node_a) or !node_id_to_index.count(node_b)) {
      return false;
    }

    Index a = node_id_to_index.at(node_a);
    Index b = node_id_to_index.at(node_b);

    bool removed = edges[a].erase(b);
    assert(edges[b].erase(a) == removed);

    _number_of_edges -= removed;
    return removed;
  }

  struct vertex_output_visitor : public planar_face_traversal_visitor {
    const std::vector<TNode> &nodes;
    std::vector<std::vector<TNode>> faces;

    vertex_output_visitor(const std::vector<TNode> &nodes) : nodes(nodes) {}

    void begin_face() { faces.emplace_back(); }

    template <typename Vertex> void next_vertex(Vertex v) {
      faces.back().push_back(nodes[v]);
    }
  };

  std::vector<std::vector<TNode>> find_all_faces() {
    PlanarGraph g(nodes.size());

    for (Index from = 0; from < nodes.size(); from++) {
      for (const auto &to : edges[from]) {
        if (from < to) {
          boost::add_edge(from, to, g);
        }
      }
    }

    // Initialize the interior edge index
    property_map<PlanarGraph, edge_index_t>::type e_index = get(edge_index, g);
    graph_traits<PlanarGraph>::edges_size_type edge_count = 0;
    graph_traits<PlanarGraph>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei)
      put(e_index, *ei, edge_count++);

    // Test for planarity - we know it is planar, we just want to
    // compute the planar embedding as a side-effect
    typedef std::vector<graph_traits<PlanarGraph>::edge_descriptor> vec_t;
    std::vector<vec_t> embedding(num_vertices(g));
    if (boyer_myrvold_planarity_test(boyer_myrvold_params::graph = g,
                                     boyer_myrvold_params::embedding =
                                         &embedding[0])) {
      std::cout << "Input graph is planar" << std::endl;
    } else {
      throw std::runtime_error("Input graph is not planar");
    }

    vertex_output_visitor v_vis(nodes);
    planar_face_traversal(g, &embedding[0], v_vis);

    return v_vis.faces;
  }

  size_t number_of_edges() const { return _number_of_edges; }

  size_t number_of_nodes() const { return nodes.size(); }

  std::string info() {
    std::stringstream ss;
    ss << "(Nodes: " << number_of_nodes() << ", Edges: " << number_of_edges()
       << ")";
    return ss.str();
  }
};

TGraph read_graph_from_json(const std::string &path) {
  if (path.empty()) {
    throw std::runtime_error("Empty path");
  }

  std::stringstream buffer;
  std::ifstream ifs(path);
  buffer << ifs.rdbuf();
  std::string js = buffer.str();

  rapidjson::Document json_input;

  if (json_input.Parse(js.c_str()).HasParseError()) {
    std::string error_msg =
        std::string(rapidjson::GetParseError_En(json_input.GetParseError())) +
        " (offset: " + std::to_string(json_input.GetErrorOffset()) + ")";
    throw std::runtime_error(error_msg);
  }

  TGraph graph;

  if (!json_input.HasMember("elements")) {
    throw std::runtime_error("No \"elements\" in JSON");
  }

  for (const auto &element : json_input["elements"].GetArray()) {
    const std::string type = element["type"].GetString();
    if (type == "node") {
      auto id = element["id"].GetUint64();
      auto lat = element["lat"].GetDouble();
      auto lng = element["lon"].GetDouble();
      graph.add_node(id, lat, lng);
    }
  }

  for (const auto &element : json_input["elements"].GetArray()) {
    const std::string type = element["type"].GetString();
    if (type == "way") {
      auto nodes = element["nodes"].GetArray();
      for (rapidjson::SizeType i = 0; i < nodes.Size() - 1; i++) {
        graph.add_edge(nodes[i].GetUint64(), nodes[i + 1].GetUint64());
      }
    }
  }

  return graph;
}

void save_result(const std::vector<std::vector<TNode>> &faces,
                 const std::string &output_file) {

  rapidjson::Document json_output(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType &allocator = json_output.GetAllocator();
  json_output.AddMember("type", "FeatureCollection", allocator);
  auto features = rapidjson::Value(rapidjson::kArrayType);

  // Create geojson
  int index = 0;
  for (auto &face : faces) {
    index++;
    rapidjson::Value feature(rapidjson::kObjectType);
    feature.AddMember("type", "Feature", allocator);
    auto properties = rapidjson::Value(rapidjson::kObjectType);
    properties.AddMember("color", index, allocator);
    feature.AddMember("properties", properties, allocator);

    auto geometry = rapidjson::Value(rapidjson::kObjectType);
    geometry.AddMember("type", "Polygon", allocator);
    auto coordinates = rapidjson::Value(rapidjson::kArrayType);
    auto coordinate = rapidjson::Value(rapidjson::kArrayType);
    for (auto &node : face) {
      auto point = rapidjson::Value(rapidjson::kArrayType);
      point.PushBack(node.lng, allocator);
      point.PushBack(node.lat, allocator);
      coordinate.PushBack(point, allocator);
    }
    auto point = rapidjson::Value(rapidjson::kArrayType);
    point.PushBack(face[0].lng, allocator);
    point.PushBack(face[0].lat, allocator);
    coordinate.PushBack(point, allocator);
    coordinates.PushBack(coordinate, allocator);
    geometry.AddMember("coordinates", coordinates, allocator);
    feature.AddMember("geometry", geometry, allocator);
    features.PushBack(feature, allocator);
  }
  json_output.AddMember("features", features, allocator);

  // Rapidjson writing process.
  rapidjson::StringBuffer s;
  rapidjson::Writer<rapidjson::StringBuffer> r_writer(s);
  json_output.Accept(r_writer);

  // Log to file.

  if (output_file.empty()) {
    std::cout << s.GetString() << std::endl;
    return;
  }

  std::ofstream out_stream(output_file, std::ofstream::out);
  out_stream << s.GetString();
  out_stream.close();
  std::cout << "Output written to " << output_file << std::endl;
}

// Save graph in geojson format
void save_graph(const TGraph &graph, const std::string &output_file) {
  rapidjson::Document json_output(rapidjson::kObjectType);
  rapidjson::Document::AllocatorType &allocator = json_output.GetAllocator();
  json_output.AddMember("type", "FeatureCollection", allocator);
  auto features = rapidjson::Value(rapidjson::kArrayType);

  // Create geojson
  for (auto &node : graph.nodes) {
    rapidjson::Value feature(rapidjson::kObjectType);
    feature.AddMember("type", "Feature", allocator);
    auto properties = rapidjson::Value(rapidjson::kObjectType);
    properties.AddMember("id", node.id, allocator);
    properties.AddMember("type", "node", allocator);
    feature.AddMember("properties", properties, allocator);

    auto geometry = rapidjson::Value(rapidjson::kObjectType);
    geometry.AddMember("type", "Point", allocator);
    auto coordinates = rapidjson::Value(rapidjson::kArrayType);
    coordinates.PushBack(node.lng, allocator);
    coordinates.PushBack(node.lat, allocator);
    geometry.AddMember("coordinates", coordinates, allocator);
    feature.AddMember("geometry", geometry, allocator);
    features.PushBack(feature, allocator);
  }

  // Add edges
  for (size_t i = 0; i < graph.nodes.size(); i++) {
    for (const auto &j : graph.edges[i]) {
      rapidjson::Value feature(rapidjson::kObjectType);
      feature.AddMember("type", "Feature", allocator);
      auto properties = rapidjson::Value(rapidjson::kObjectType);
      properties.AddMember("id", i, allocator);
      properties.AddMember("type", "way", allocator);
      properties.AddMember("nodes", rapidjson::Value(rapidjson::kArrayType),
                           allocator);
      properties["nodes"].PushBack(graph.nodes[i].id, allocator);
      properties["nodes"].PushBack(graph.nodes[j].id, allocator);
      feature.AddMember("properties", properties, allocator);

      auto geometry = rapidjson::Value(rapidjson::kObjectType);
      geometry.AddMember("type", "LineString", allocator);
      auto coordinates = rapidjson::Value(rapidjson::kArrayType);
      auto point = rapidjson::Value(rapidjson::kArrayType);
      point.PushBack(graph.nodes[i].lng, allocator);
      point.PushBack(graph.nodes[i].lat, allocator);
      coordinates.PushBack(point, allocator);
      point = rapidjson::Value(rapidjson::kArrayType);
      point.PushBack(graph.nodes[j].lng, allocator);
      point.PushBack(graph.nodes[j].lat, allocator);
      coordinates.PushBack(point, allocator);
      geometry.AddMember("coordinates", coordinates, allocator);
      feature.AddMember("geometry", geometry, allocator);
      features.PushBack(feature, allocator);
    }
  }

  json_output.AddMember("features", features, allocator);

  // Rapidjson writing process.
  rapidjson::StringBuffer s;
  rapidjson::Writer<rapidjson::StringBuffer> r_writer(s);
  json_output.Accept(r_writer);

  // Log to file.

  if (output_file.empty()) {
    std::cout << s.GetString() << std::endl;
    return;
  }

  std::ofstream out_stream(output_file, std::ofstream::out);
  out_stream << s.GetString();
  out_stream.close();
  std::cout << "Logs written to " << output_file << std::endl;
}

void remove_leafs(TGraph &graph) {
  size_t nu_edges_before = graph.number_of_edges();
  size_t nu_nodes_before = graph.number_of_nodes();
  std::queue<Index> leafs;

  for (Index i = 0; i < graph.nodes.size(); i++) {
    if (graph.edges[i].size() == 1) {
      leafs.push(i);
    }
  }

  while (!leafs.empty()) {
    Index leaf = leafs.front();
    leafs.pop();
    if (graph.edges[leaf].size() != 1) {
      continue;
    }
    Index parent = *graph.edges[leaf].begin();
    graph.remove_edge(graph.nodes[leaf].id, graph.nodes[parent].id);
    if (graph.edges[parent].size() == 1) {
      leafs.push(parent);
    }
  }

  // Normolize graph
  TGraph reduced_graph;

  for (Index i = 0; i < graph.nodes.size(); i++) {
    if (graph.edges[i].empty()) {
      continue;
    }

    reduced_graph.add_node(graph.nodes[i].id, graph.nodes[i].lat,
                           graph.nodes[i].lng);
  }

  for (Index i = 0; i < graph.nodes.size(); i++) {
    for (const auto &j : graph.edges[i]) {
      reduced_graph.add_edge(graph.nodes[i].id, graph.nodes[j].id);
    }
  }

  graph = reduced_graph;

  size_t nu_edges_after = graph.number_of_edges();
  size_t nu_nodes_after = graph.number_of_nodes();

  std::cout << "Removed "
            << double(nu_edges_before - nu_edges_after) / (nu_edges_before)
            << " \% edges." << std::endl;
  std::cout << "Removed "
            << double(nu_nodes_before - nu_nodes_after) / (nu_nodes_before)
            << " \% nodes." << std::endl;
}

int main(int argc, char **argv) {

  // Read input file name from command line arguments.
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <input_file> <output_file>"
              << std::endl;
    return 1;
  }

  std::string input_file = argv[1];
  std::string output_file = argv[2];

  auto graph = read_graph_from_json(input_file);
  std::cout << "Initial graph: " << graph.info() << std::endl;

  remove_leafs(graph);

  std::cout << "Finding faces... " << graph.info() << std::endl;
  auto faces = graph.find_all_faces();

  save_result(faces, output_file);
  save_graph(graph, "logs/graph.geojson");

  return 0;
}
