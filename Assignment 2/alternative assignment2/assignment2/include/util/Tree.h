#ifndef TREE_H
#define TREE_H

#include <iostream>
#include <vector>

template <typename T>
class TreeNode {
public:
    T data;
    std::vector<TreeNode<T>*> children;

    TreeNode(const T& value);
    ~TreeNode();
};

template <typename T>
class Tree {
private:
    TreeNode<T>* root;

public:
    Tree();
    ~Tree();

    inline TreeNode<T>* get_root() { return root; }
    TreeNode<T>* add_root(const T& value);
    TreeNode<T>* add_node(const T& value, TreeNode<T>* parent);
    T visit_node(TreeNode<T>* node);
    TreeNode<T>* remove_node(TreeNode<T>* node);
    std::vector<TreeNode<T>*> get_children(TreeNode<T>* node) const;
    size_t size() const;
    bool is_empty() const;

private:
    size_t count_nodes(TreeNode<T>* node) const;
    TreeNode<T>* find_parent(TreeNode<T>* current, TreeNode<T>* target) const;
};

// Function declaration for depth-first search (with callback function to apply to each visited node)
template <typename T>
void depth_first_search(TreeNode<T>* startNode, void (*visit)(T));
template <typename T>
void depth_first_search(TreeNode<T>* startNode, void (*visit)(T&));
// Recursive helper function for depth-first search
template <typename T>
void depth_first_search_recursive(TreeNode<T>* currentNode, void (*visit)(T));
template <typename T>
void depth_first_search_recursive(TreeNode<T>* currentNode, void (*visit)(T&));

#endif // TREE_H
