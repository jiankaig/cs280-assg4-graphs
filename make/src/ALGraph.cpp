#include "ALGraph.h"

 ALGraph::ALGraph(unsigned size){
   (void)size;
 }

ALGraph::~ALGraph(void){

}

void ALGraph::AddDEdge(unsigned source, unsigned destination, unsigned weight){
  (void)source;
  (void)destination;
  (void)weight;
}

void ALGraph::AddUEdge(unsigned node1, unsigned node2, unsigned weight){
  (void)node1;
  (void)node2;
  (void)weight;
}

std::vector<DijkstraInfo> ALGraph::Dijkstra(unsigned start_node) const{
  (void)start_node;
  std::vector<DijkstraInfo> ret;
  return ret;
}

ALIST ALGraph::GetAList(void) const{
  ALIST ret;
  return ret;
}