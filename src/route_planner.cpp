#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // find the closest nodes to the starting and ending coordinates.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Calculate the distance to the end_node for the h value.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*(this->end_node));
}


// Expanding the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();
  
    // Fill neighbors attributes
    for(auto n: current_node->neighbors){
        if(!n->visited){
            n->parent = current_node;
            n->g_value = current_node->g_value + current_node->distance(*n);
            n->h_value = RoutePlanner::CalculateHValue(n);
            n->visited = true;
            this->open_list.push_back(n);
        }
    }
}


// Get the next node to process
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value.
    std::sort(this->open_list.begin(),
         this->open_list.end(),
         [](RouteModel::Node const *a, RouteModel::Node const *b) {return ((a->g_value + a->h_value) > (b->g_value + b->h_value));});
    
    // Create a pointer to the node in the list with the lowest sum.
    auto lowest = this->open_list.back();
  
    // Remove that node from the open_list.
    this->open_list.pop_back();
  
    // Return the pointer.
    return lowest;
}


// Return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  
    // Push current node to the path and calculate distance to parent
    path_found.push_back(*current_node);
    if (current_node->parent != nullptr)
        distance += current_node->distance(*current_node->parent);
  
    // Iteratively follow the chain of parents
    auto chained_node = current_node->parent;
    while (chained_node != nullptr){
        path_found.push_back(*chained_node);
        // Add distance from the node to its parent
        if (chained_node->parent != nullptr)
            distance += chained_node->distance(*chained_node->parent);
        chained_node = chained_node->parent;
    }
  
    // Reverse the path
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  
    // Initialize with the start_node
    current_node = this->start_node;
    current_node->parent = nullptr;
    current_node->g_value = 0;
    current_node->h_value = RoutePlanner::CalculateHValue(current_node);
    current_node->visited = true;
    this->open_list.push_back(current_node);

    // Iterate over the open_list of nodes
    while(this->open_list.size() > 0){
        // Get the current node
        current_node = this->NextNode();
      
        // The end node is reached
        if(current_node == end_node){
            // Construct the final path and return
            this->m_Model.path = ConstructFinalPath(current_node);
            break;
        }
      
        // Expand neighbors
        this->AddNeighbors(current_node);
    }
}