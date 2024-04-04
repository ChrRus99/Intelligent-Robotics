#include "util/Tree.h"
#include "util/Point.h"
#include "models/NavigationModels.h"
#include <algorithm>

using namespace std;
using namespace Point;
using namespace NavigationModels;

template <typename T>
TreeNode<T>::TreeNode(const T& value) : data(value) {}

template <typename T>
TreeNode<T>::~TreeNode() {
    for (TreeNode* child : children) {
        delete child;
    }
}

template <typename T>
Tree<T>::Tree() : root(nullptr) {}

template <typename T>
Tree<T>::~Tree() {
    delete root;
}

template <typename T>
TreeNode<T>* Tree<T>::add_root(const T& value) {
    if (root == nullptr) {
        root = new TreeNode<T>(value);
        return root;
    } else {
        throw runtime_error("Error: Root already exists in the tree.");
    }
}

template <typename T>
TreeNode<T>* Tree<T>::add_node(const T& value, TreeNode<T>* parent) {
    if (parent == nullptr) {
        throw invalid_argument("Error: Parent node cannot be null.");
    }

    TreeNode<T>* newNode = new TreeNode<T>(value);
    parent->children.push_back(newNode);
    return newNode;
}

template <typename T>
T Tree<T>::visit_node(TreeNode<T>* node) {
    if (node != nullptr) {
        return node->data;
    } else {
        throw invalid_argument("Error: Cannot visit a null node.");
    }
}

template <typename T>
TreeNode<T>* Tree<T>::remove_node(TreeNode<T>* node) {
    if (node != nullptr) {
        TreeNode<T>* parent = find_parent(root, node);
        if (parent != nullptr) {
            auto it = find(parent->children.begin(), parent->children.end(), node);
            if (it != parent->children.end()) {
                parent->children.erase(it);
                node->children.clear(); // Disconnect the deleted node from its children
                return node;
            }
        } else {
            throw logic_error("Error: Cannot remove the root node.");
        }
    }
    return nullptr; // Node not found
}

template <typename T>
vector<TreeNode<T>*> Tree<T>::get_children(TreeNode<T>* node) const {
    return (node != nullptr) ? node->children : vector<TreeNode<T>*>();
}

template <typename T>
size_t Tree<T>::size() const {
    return count_nodes(root);
}

template <typename T>
bool Tree<T>::is_empty() const {
    return (root == nullptr);
}

template <typename T>
size_t Tree<T>::count_nodes(TreeNode<T>* node) const {
    if (node == nullptr) {
        return 0;
    }

    size_t count = 1;
    for (TreeNode<T>* child : node->children) {
        count += count_nodes(child);
    }
    return count;
}

template <typename T>
TreeNode<T>* Tree<T>::find_parent(TreeNode<T>* current, TreeNode<T>* target) const {
    if (current == nullptr || current->children.empty()) {
        return nullptr;
    }

    for (TreeNode<T>* child : current->children) {
        if (child == target) {
            return current;
        } else {
            TreeNode<T>* found = find_parent(child, target);
            if (found != nullptr) {
                return found;
            }
        }
    }

    return nullptr;
}

template <typename T>
void depth_first_search(TreeNode<T>* startNode, void (*visit)(T)) {
    if (startNode != nullptr && visit != nullptr) {
        depth_first_search_recursive(startNode, visit);
    }
}

template <typename T>
void depth_first_search_recursive(TreeNode<T> *currentNode, void (*visit)(T)) {
    if (currentNode == nullptr) {
        return;
    }

    // Visit the current node
    visit(currentNode->data);

    // Recursively visit children
    for (TreeNode<T>* child : currentNode->children) {
        depth_first_search_recursive(child, visit);
    }
}

template <typename T>
void depth_first_search(TreeNode<T>* startNode, void (*visit)(T&)) {
    if (startNode != nullptr && visit != nullptr) {
        depth_first_search_recursive(startNode, visit);
    }
}

template <typename T>
void depth_first_search_recursive(TreeNode<T> *currentNode, void (*visit)(T&)) {
    if (currentNode == nullptr) {
        return;
    }

    // Visit the current node
    visit(currentNode->data);

    // Recursively visit children
    for (TreeNode<T>* child : currentNode->children) {
        depth_first_search_recursive(child, visit);
    }
}


// Explicit instantiation of the template for supported types
template void depth_first_search<NavigationPoseProperties>(
    TreeNode<NavigationPoseProperties>* startNode,
    void (*visit)(NavigationPoseProperties)
);

template void depth_first_search<NavigationPoseProperties>(
    TreeNode<NavigationPoseProperties>* startNode,
    void (*visit)(NavigationPoseProperties&)
);

// Explicit instantiation of the template for supported types
template class Tree<NavigationPoseProperties>;