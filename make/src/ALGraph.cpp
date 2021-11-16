#include "ALGraph.h"

ALGraph::ALGraph(unsigned size) : size_(size){
  for(unsigned i = 0; i<size ; i++){
    DijkstraInfo temp;
    std::stack<unsigned> tempStack;
    // std::vector<unsigned> path_;
    // temp.path = path_;
    std::vector<AdjacencyInfo> AdjList_;
    std::vector<AdjInfo> AdjInfoList_;
    graph_.insert({i, AdjInfoList_});
    ALIST_.push_back(AdjList_);
    DistArray.push_back(INFINITY_);
    SelectedDistArray.push_back(false);
    DijkstraList.push_back(temp);
    pathList_.push_back(tempStack);
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
  
  // 1. init DistArray elements with starting vertex
  DistArray[start_node-1] = 0; 
  SelectedDistArray[start_node-1] = true;
  for(unsigned i=0;i<graph_[start_node-1].size();i++){
    checkAdjNodes(graph_[start_node-1], &DistArray[start_node-1], start_node-1);
    
  } 
  DijkstraList[start_node-1].cost = 0;
  DijkstraList[start_node-1].path.push_back(start_node);

  // 2. select node with smallest distance that has not been selected

  // 3. update DistArray (those unselected ones only?)
  for(unsigned i=1;i<graph_.size();i++){
      unsigned minNode = SelectMinNode(DistArray, SelectedDistArray);
      checkAdjNodes(graph_[minNode], &DistArray[minNode], minNode);
  } 
  // 4. repeat 2 & 3 size_-1 times

  //compensate paths with start ndoe
  for(unsigned i=0;i<graph_.size();i++){
    if(i!=start_node-1)
      DijkstraList[i].path.insert(DijkstraList[i].path.begin(), start_node);
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

void ALGraph::checkAdjNodes(std::vector<AdjInfo>& AdjInfoList, unsigned* DistArrayElement, int index)const{
  //check adjacent nodes
  for(unsigned j=0; j<AdjInfoList.size(); j++){
    if(DistArray[AdjInfoList[j].node->id-1] == INFINITY_ || 
      *DistArrayElement + AdjInfoList[j].weight < DistArray[AdjInfoList[j].node->id-1])
    {
      //graph_[i][j].node.id; // <- adjacent node
      DistArray[AdjInfoList[j].node->id-1] = *DistArrayElement + AdjInfoList[j].weight;
      DijkstraList[AdjInfoList[j].node->id-1].cost = DistArray[AdjInfoList[j].node->id-1];
      DijkstraList[AdjInfoList[j].node->id-1].path = DijkstraList[index].path;
      DijkstraList[AdjInfoList[j].node->id-1].path.push_back(AdjInfoList[j].node->id);
    }
    // else
    //   DijkstraList[i].cost = DistArray[i];
  }
}

unsigned ALGraph::SelectMinNode(std::vector<unsigned>& DistArray, std::vector<bool>& SelectedDistArray)const{
    // Initialize min value
    unsigned min = INFINITY_;
    unsigned min_index;
 
    for (unsigned v = 0; v < DistArray.size(); v++){
        if (SelectedDistArray[v] == false && DistArray[v] <= min){
            min = DistArray[v], min_index = v;
        }
    }
    SelectedDistArray[min_index] = true;
    return min_index;
}


////////////////////////////////////////////////////////////////////////
ALGraph::AdjInfo::AdjInfo() : node(new GNode()), weight(0), cost(0){}

bool ALGraph::AdjInfo::operator<(const AdjInfo& rhs) const{
  if(this->weight < rhs.weight)
    return true;
  else if(this->weight == rhs.weight && this->node->id < rhs.node->id)
    return true;
  return false;
}
bool ALGraph::AdjInfo::operator>(const AdjInfo& rhs) const{
  if(this->weight > rhs.weight)
    return true;
  else if(this->weight == rhs.weight && this->node->id > rhs.node->id)
    return true;
  return false;
}
