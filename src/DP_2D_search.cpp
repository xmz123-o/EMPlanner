#include "DP_2D_search.h"

int main() {
    int num_layers = 6;
    int num_cols = 5;

    // Build Graph
    std::vector<std::vector<Node>> graph;
    for(int l = 0;l < num_layers; ++l) {
        std::vector<Node>  row;
        double s = l * 5.0;
        for(int c = 0;c < num_cols; ++c) {
            row.emplace_back(l,c);
        }
        graph.push_back(row);
    }

    //Start Node
    int start_layer = 0;
    int start_col = 2;
    graph[start_layer][start_col].cost = 0.0;

    //Obstacles
    graph[1][2].is_obstacle = true;
    graph[2][2].is_obstacle = true;
    graph[3][1].is_obstacle = true;
    graph[3][0].is_obstacle = true;
    graph[4][3].is_obstacle = true;

    //Map
    std::cout <<"Graph:\n";
    for(int l = 0; l <num_layers; ++l) {
        for(int c = 0; c < num_cols; ++c) {
            std::cout << (graph[l][c].is_obstacle ? "X " : "O ");
        }
        std::cout << std::endl;
    }

    // DP Search
    for(int l = 0; l < num_layers - 1; ++l) {
        for(int c = 0; c < num_cols; ++c) {
            Node& current = graph[l][c];

            if(std::isinf(current.cost)) {
                continue; // Skip unreachable nodes
            }

            for(int next_c = 0; next_c < num_cols; ++next_c) {
                Node& next = graph[l + 1][next_c];
                if(next.is_obstacle)
                continue; // Skip obstacles

                //Transition cost
                double transition_cost = std::abs(next_c - c);
                double new_cost = current.cost + transition_cost;

                if(new_cost < next.cost) {
                    next.cost = new_cost;
                    next.parent = &current;
                }
            }

        }
    }

    //Find best node in the last layer
    Node* best = nullptr;
    double min_cost = std::numeric_limits<double>::infinity();
    for(int c = 0; c < num_cols; ++c) {
        Node& node = graph[num_layers - 1][c];
        if(node.cost < min_cost) {
            min_cost = node.cost;
            best = &node;
        }

    }

    //Bachtrack to find best path
    std::vector<Node*> path;
    while(best != nullptr) {
        path.push_back(best);
        best = best -> parent;
    }

    std::reverse(path.begin(), path.end());
    std::cout <<"Best path: \n";
    for(const auto& node : path) {
        std::cout << "Layer: "<< node->layer <<", Col: "<<node->col <<", Cost: "<<node->cost << std::endl;
    }

    return 0;
}