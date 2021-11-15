#include "ALGraph.h"

ALGraph::ALGraph(unsigned size) : size_(size){
  for(unsigned i = 0; i<size ; i++){
    DijkstraInfo temp;
    std::vector<AdjacencyInfo> AdjList_;
    std::vector<AdjInfo> AdjInfoList_;
    graph_.insert({i, AdjInfoList_});
    ALIST_.push_back(AdjList_);
    DistArray.push_back(INFINITY_);
    DijkstraList.push_back(temp);
    // sptSet.push_back(false);// sptSet[i] will be true if vertex i is included in shortest path tree
  }
}

ALGraph::~ALGraph(void){

}

void ALGraph::AddDEdge(unsigned source, unsigned destination, unsigned weight){
  AddEdge(source, destination, weight);
}

void ALGraph::AddUEdge(unsigned node1, unsigned node2, unsigned weight){
  AddEdge(node1, node2, weight);
  AddEdge(node2, node1, weight);
}

std::vector<DijkstraInfo> ALGraph::Dijkstra(unsigned start_node) const{
  DistArray[start_node-1] = 0; //basecase
  DijkstraList[0].cost = 0; //basecase
  // sptSet[start_node] = true; //basecase
  
  // 1. init DistArray elements with starting vertex
  // 2. select node with smallest distance that has not been selected
  // 3. update DistArray (those unselected ones only?)
  // 4. repeat 2 & 3 size_-1 times

  //update DistArray for particular node
  for(unsigned i=0; i < size_; i++){
    std::vector<unsigned> path;
    //check adjacent nodes
    for(unsigned j=0; j<graph_[i].size(); j++){
      if(DistArray[graph_[i][j].node->id-1] == INFINITY_ || 
        DistArray[i] + graph_[i][j].weight < DistArray[graph_[i][j].node->id-1])
      {
        //graph_[i][j].node.id; // <- adjacent node
        DistArray[graph_[i][j].node->id-1] = DistArray[i] + graph_[i][j].weight;
        DijkstraList[graph_[i][j].node->id-1].cost = DistArray[graph_[i][j].node->id-1];
      }
      else
        DijkstraList[i].cost = DistArray[i];
    }
    // DijkstraList[i].cost = DistArray[i];
    DijkstraList[i].path = path;
  }

  return DijkstraList; //returns a list of DijkstraInfo
}

ALIST ALGraph::GetAList(void) const{
  return ALIST_;
}

void ALGraph::AddEdge(unsigned node1, unsigned node2, unsigned weight){
  AdjInfo temp;
  temp.node->id = node2;
  temp.weight = weight;

  // load list of adjacent vertices into priority queue
  std::priority_queue<AdjInfo> pq;
  // std::priority_queue<AdjInfo, std::vector, std::greater> pq;
  
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

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
// unsigned ALGraph::minDistance(std::vector<int>& dist, std::vector<bool>& sptSet) 
// {
   
//     // Initialize min value
//     unsigned min, min_index = -1;

//     for (unsigned v = 0; v < size_; v++)
//         if (sptSet[v] == false && dist[v] <= min)
//             min = dist[v], min_index = v;
 
//     return min_index;
// }

// void ALGraph::helperfunction(){
//   DistArray[0] = 0; //basecase
//   sptSet[0] = true; //basecase
// }

////////////////////////////////////////////////////////////////////////
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
