#include "ALGraph.h"

ALGraph::ALGraph(unsigned size){
  for(unsigned i = 0; i<size ; i++){
    std::vector<AdjacencyInfo> AdjList_;
    std::vector<AdjInfo> AdjInfoList_;
    graph_.insert({i, AdjInfoList_});
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
  AdjInfo temp;
  temp.node->id = node2;
  temp.weight = weight;

  // node1's list of adjacent vertices
  // auto node1AdjList = ALIST_[node1-1];

  // load list of adjacent vertices into priority queue
  // std::priority_queue<AdjInfo, vector<AdjInfo>, greater<AdjInfo> > pq;
  std::priority_queue<AdjInfo> pq;
  for(unsigned i=0; i < graph_[node1-1].size(); i++){
    pq.push(graph_[node1-1][i]);
  }

  // insert new vertice into queue
  pq.push(temp);

  // store/overwite list of adjacent vertices in node1
  writeToList(graph_[node1-1], ALIST_[node1-1], pq);
}

void ALGraph::writeToList(std::vector<ALGraph::AdjInfo>& graphElement, 
  std::vector<AdjacencyInfo>& list, std::priority_queue<ALGraph::AdjInfo> pq){
  graphElement.clear();
  list.clear();
  std::stack<AdjInfo> s;
  while(!pq.empty()){
    s.push(pq.top());
    pq.pop();
  }

  while(!s.empty()){
    AdjacencyInfo temp;
    temp.id = s.top().node->id;
    temp.weight = s.top().weight;
    graphElement.push_back(s.top());
    list.push_back(temp);
    s.pop();
  }
}

ALGraph::AdjInfo::AdjInfo() : node(new GNode()), weight(0), cost(0){}

bool ALGraph::AdjInfo::operator<(const AdjInfo& rhs) const{
  if(this->weight < rhs.weight)
    return true;
  return false;
}
bool ALGraph::AdjInfo::operator>(const AdjInfo& rhs) const{
  if(this->weight > rhs.weight)
    return true;
  return false;
}
