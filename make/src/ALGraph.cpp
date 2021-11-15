#include "ALGraph.h"
#include <queue>

ALGraph::ALGraph(unsigned size){
  for(unsigned i = 0; i<size ; i++){
    std::vector<AdjacencyInfo> AdjList_;
    // graph_.insert({i, AdjList_});
    ALIST_.push_back(AdjList_);
  }
}

ALGraph::~ALGraph(void){

}

void ALGraph::AddDEdge(unsigned source, unsigned destination, unsigned weight){
  (void)source;
  (void)destination;
  (void)weight;
}

void ALGraph::AddUEdge(unsigned node1, unsigned node2, unsigned weight){
  AddEdge(node1, node2, weight);
  AddEdge(node2, node1, weight);
}

std::vector<DijkstraInfo> ALGraph::Dijkstra(unsigned start_node) const{
  (void)start_node;
  std::vector<DijkstraInfo> ret;
  return ret;
}

ALIST ALGraph::GetAList(void) const{
  return ALIST_;
}

void ALGraph::AddEdge(unsigned node1, unsigned node2, unsigned weight){
  AdjacencyInfo temp;
  temp.id = node2;
  temp.weight = weight;

  // node1's list of adjacent vertices
  // auto node1AdjList = ALIST_[node1-1];

  // load list of adjacent vertices into priority queue
  // std::priority_queue<AdjacencyInfo> pq;
  // for(unsigned i=0; i < ALIST_[node1-1].size(); i++){
  //   pq.push(ALIST_[node1-1][i]);
  // }

  // insert new vertice into queue

  // store/overwite list of adjacent vertices in node1

  ALIST_[node1-1].push_back(temp);
}