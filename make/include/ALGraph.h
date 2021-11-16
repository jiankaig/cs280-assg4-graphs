//---------------------------------------------------------------------------
#ifndef ALGRAPH_H
#define ALGRAPH_H
//---------------------------------------------------------------------------
#include <vector>
#include <map> //self add
#include <queue> //self add
#include <stack> //self add
#include <algorithm> //self add

struct DijkstraInfo
{
  unsigned cost;
  std::vector<unsigned> path;
};

struct AdjacencyInfo
{
  unsigned id;
  unsigned weight;
};

typedef std::vector<std::vector<AdjacencyInfo> > ALIST;

class ALGraph
{
  public:
    ALGraph(unsigned size);
    ~ALGraph(void);
    void AddDEdge(unsigned source, unsigned destination, unsigned weight);
    void AddUEdge(unsigned node1, unsigned node2, unsigned weight);

    std::vector<DijkstraInfo> Dijkstra(unsigned start_node) const;
    ALIST GetAList(void) const;
        
  private:
    
    // An EXAMPLE of some other classes you may want to create and 
    // implement in ALGraph.cpp
    class GNode{
      public:
        unsigned id;
        GNode(): id(-1){} 
        GNode(unsigned node_) : id(node_){}
    }; 
    class GEdge;
    struct AdjInfo
    {
      GNode *node;
      unsigned weight;
      unsigned cost;
      AdjInfo();
      bool operator<(const AdjInfo& rhs) const;
      bool operator>(const AdjInfo& rhs) const;
    };
    
    // Other private fields and methods
    mutable std::map<unsigned, std::vector<AdjInfo>> graph_;
    ALIST ALIST_;
    mutable std::vector<DijkstraInfo> DijkstraList;
    mutable std::vector<unsigned> DistArray;
    mutable std::vector<bool> SelectedDistArray;
    const unsigned INFINITY_ =static_cast<unsigned>(-1);
    unsigned size_;
    std::vector<std::stack<unsigned>> pathList_;

    void AddEdge(unsigned node1, unsigned node2, unsigned weight);
    void writeToList(std::vector<ALGraph::AdjInfo>& graphElement, 
      std::vector<AdjacencyInfo>& list, std::priority_queue<AdjInfo> pq);
    void checkAdjNodes(std::vector<AdjInfo>& AdjInfoList,
      unsigned* DistArrayElement, int index)const;
    unsigned SelectMinNode(std::vector<unsigned>& DistArray, std::vector<bool>& SelectedDistArray)const;
};

#endif
