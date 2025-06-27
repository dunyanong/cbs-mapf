  /*******************************************
   * Author: Zhongqiang Richard Ren. 
   * All Rights Reserved. 
   *******************************************/

  #include "avltree.hpp"
  #include "graph.hpp"
  #include "vec_type.hpp"
  #include "signal.h"
  
  namespace raplab{
  
  // Overload the << operator to print AVLNode information
  std::ostream& operator<<(std::ostream& os, const AVLNode& n) {
    os << "AVLNode(id:" << n.id << ",hasLeft:" << int(n.left!=NULL) << ",hasRight:" << int(n.right!=NULL) << ",h:" << n.h << ")";
    return os;
  };
  
  // Create a new AVLNode with given ID
  AVLNode* NewAVLNode(long id) {
    AVLNode *n = new AVLNode();
    n->id = id;
    n->left = NULL;
    n->right = NULL;
    n->h = 1; // Initial height is 1 for new node
    return n;
  };
  
  // Right rotation operation to balance the tree
  AVLNode *RightRotate(AVLNode *y) {
    // find related data
    AVLNode *x = y->left;
    AVLNode *t2 = x->right;
  
    // perform rotation
    x->right = y;
    y->left = t2;
    
    // update heights after rotation
    y->h = Max( H(y->left), H(y->right) ) + 1;
    x->h = Max( H(x->left), H(x->right) ) + 1;
    return x; // new root after rotation
  };
  
  // Left rotation operation to balance the tree
  AVLNode *LeftRotate(AVLNode *x) {
    // find related data
    AVLNode *y = x->right;
    AVLNode *t2 = y->left;
  
    // perform rotation
    y->left = x;
    x->right = t2;
    
    // update heights after rotation
    x->h = Max( H(x->left), H(x->right) ) + 1;
    y->h = Max( H(y->left), H(y->right) ) + 1;
  
    return y; // new root after rotation
  };
  
  // Calculate balance factor of a node (difference in heights of left and right subtrees)
  int GetBalanceFactor(AVLNode *n) {
    if (n == NULL) {return 0;}
    return H(n->left) - H(n->right);
  };
  
  
  // AVLTree class implementation
  
  // Constructor
  template <typename DataType>
  AVLTree<DataType>::AVLTree() {};
  
  // Destructor - deletes all nodes in the tree
  template <typename DataType>
  AVLTree<DataType>::~AVLTree() {
    _deleteAll(_root); // need to delete/free all nodes.
  };
    
  // Add a new node with key k and optional id
  template <typename DataType>
  void AVLTree<DataType>::Add(DataType k, long id) {
    _root = _insert(_root, k, id);
    return;
  };
  
  // Print the tree in pre-order traversal
  template <typename DataType>
  void AVLTree<DataType>::Print() {
    std::cout << "AVL-tree:{";
    _PreOrder(_root);
    std::cout << "}" << std::endl;
    return;
  };
  
  // Find a node with key k
  template <typename DataType>
  AVLNode AVLTree<DataType>::Find(DataType k) {
    AVLNode* p = _find(_root, k);
    if (p == NULL) {
      return AVLNode(); // return empty node if not found
    }else{
      return *p; // make a copy to return
    }
  };
  
  // Find the maximum key less than k
  template <typename DataType>
  int AVLTree<DataType>::FindMaxLess(DataType k, DataType *out, bool if_equal, long *out_id) {
    long ref = -1;
    this->_findMaxLess(_root, k, &ref, if_equal);
    if (ref < 0) {
      return 0; // not found
    }
    *out = _key[ref];
    if (out_id != NULL) {
      *out_id = ref; // copy tree-node ID, if needed.
    }
    return 1; // found
  };
  
  // Find the minimum key greater than k
  template <typename DataType>
  int AVLTree<DataType>::FindMinMore(DataType k, DataType *out, bool if_equal, long *out_id) {
    long ref = -1;
    _findMinMore(_root, k, &ref, if_equal);
    if (ref < 0) {
      return 0; // not found
    }
    *out = _key[ref];
    if (out_id != NULL) {
      *out_id = ref; // copy tree-node ID, if needed.
    }
    return 1; // found
  };
  
  // Delete a node with key k
  template <typename DataType>
  void AVLTree<DataType>::Delete(DataType k) {
    _root = _delete(_root, k);
    return;
  };
  
  // Clear the entire tree
  template <typename DataType>
  void AVLTree<DataType>::Clear() {
    _key.clear(); // clear all keys
    _deleteAll(_root); // delete all nodes
    _root = NULL; 
    _id_gen = 0; // reset ID generator
    _size = 0; // reset size
    return ;
  };
  
  // Get the number of nodes in the tree
  template <typename DataType>
  size_t AVLTree<DataType>::Size() const {
    return _size;
  }; 
  
  // Convert the tree to a sorted vector (in-order traversal)
  template <typename DataType>
  void AVLTree<DataType>::ToSortedVector(
    std::vector<DataType> *out, std::vector<long> *out_id, std::unordered_set<long> *skip_set)
  {
    _InOrder2Vec(_root, out, out_id, skip_set);
    return;
  };
  
  // Pre-order traversal (used for printing)
  template <typename DataType>
  void AVLTree<DataType>::_PreOrder(AVLNode *n) {
    if(n != NULL) {
      std::cout << "[k:" << _key[n->id] << ",h:" << n->h << ",id:" << n->id << "],";
      _PreOrder(n->left);
      _PreOrder(n->right);
    }
  return;
  };
  
  // In-order traversal that stores results in vectors
  template <typename DataType>
  void AVLTree<DataType>::_InOrder2Vec(
    AVLNode *n, std::vector<DataType> *out, std::vector<long> *out_id, 
    std::unordered_set<long> *skip_set) 
  {
    if (n == NULL) {
      return;
    }
  
    _InOrder2Vec(n->left, out, out_id, skip_set);
  
    // Add to output if not in skip_set
    if ( (skip_set == NULL) || 
          (skip_set->find(n->id) == skip_set->end()) )
    {
      if (out != NULL) {
        out->push_back(_key[n->id]);
      }
      if (out_id != NULL) {
        out_id->push_back(n->id);
      }
    }
  
    _InOrder2Vec(n->right, out, out_id, skip_set);
  
    return ;
  };
  
  // Find a node with key k (recursive)
  template <typename DataType>
  AVLNode* AVLTree<DataType>::_find(AVLNode* n, DataType k) {
    if (n == NULL) {
      return n; // not found
    }
    if (k < _key[n->id]) {
      return _find(n->left, k); // search left subtree
    }else if (k > _key[n->id]) {
      return _find(n->right, k); // search right subtree
    }else {
      return n; // found it !
    }
    throw std::runtime_error( "[WARNING] AVL-tree _find reaches the end !?" );
  };
  
  // Find the maximum key less than k (helper function)
  template <typename DataType>
  void AVLTree<DataType>::_findMaxLess(AVLNode* n, DataType k, long* ref, bool if_equal) {
    if (n == NULL) {
      return;
    }
    if (_key[n->id] < k) {
      // Check if current node is better than previous best
      bool temp = (*ref == -1) || (_key[n->id] > _key[*ref]);
      if (temp) {
        *ref = n->id; // update reference
      }
      this->_findMaxLess(n->right, k, ref, if_equal); // check right subtree
      return;
    }else if (_key[n->id] > k) { 
      this->_findMaxLess(n->left, k, ref, if_equal); // check left subtree
      return;
    }else{ // _key[n->id] == k
      if (if_equal) {
        *ref = n->id; // exact match if allowed
        return;
      } else { // search left subtree for smaller values
        this->_findMaxLess(n->left, k, ref, if_equal);
        return;
      }
    }
    throw std::runtime_error( "[WARNING] AVL-tree _findMaxLess reaches the end !?" );
  };
  
  // Find the minimum key greater than k (helper function)
  template <typename DataType>
  void AVLTree<DataType>::_findMinMore(AVLNode* n, DataType k, long* ref, bool if_equal) {
    if (n == NULL) {
      return;
    }
    if (_key[n->id] > k) {
      // Check if current node is better than previous best
      bool temp = ((*ref == -1) || (_key[n->id] < _key[*ref]));
      if (temp) {
        *ref = n->id; // update reference
      }
      _findMinMore(n->left, k, ref, if_equal); // check left subtree
      return;
    }else if (_key[n->id] < k) { 
      _findMinMore(n->right, k, ref, if_equal); // check right subtree
      return;
    }else{ // _key[n->id] == k
      if (if_equal) {
        *ref = n->id; // exact match if allowed
        return;
      } else { // search right subtree for larger values
        _findMinMore(n->right, k, ref, if_equal);
        return;
      }
    }
    throw std::runtime_error( "[WARNING] AVL-tree _findMinMore reaches the end !?" );
  };
  
  // Insert a new node with key k and optional id (recursive)
  template <typename DataType>
  AVLNode* AVLTree<DataType>::_insert(AVLNode* n, DataType k, long id0) {
    // Standard BST insertion first
  
    if (n == NULL) {
      auto nn = NewAVLNode(id0);
      if (id0 < 0) {
        nn->id = _IdGen(); // generate new ID if not provided
      }
  
      #ifdef USE_STD_VECTOR_IMPL
        if (nn->id >= _key.size()){
          _key.resize(nn->id + 1);
        }
        _key[nn->id] = k; // store key
      #endif
  
      _key[nn->id] = k; // store key
      _size++;
  
      return nn;
    }
  
    // Recursive insertion
    if (k < _key[n->id]) {
      n->left = _insert(n->left, k, id0);
    }else if (k > _key[n->id]) {
      n->right = _insert(n->right, k, id0);
    }else {
      // Duplicate key - return existing node
      return n;
    }
    
    // Update height of current node
    n->h = Max( H(n->left),H(n->right) ) + 1;
    
    // Check balance factor
    int b = GetBalanceFactor(n);
  
    // Handle 4 possible rotation cases
    if (b > 1 && (n->left != NULL) && k < _key[n->left->id]) {
      // Left Left case - right rotation
      auto temp = RightRotate(n);
      if (DEBUG_AVLTREE) { _verifyTree(n);}
      return temp;
    }
    if (b < -1 && (n->right != NULL) && k > _key[n->right->id]) {
      // Right Right case - left rotation
      auto temp = LeftRotate(n);
      if (DEBUG_AVLTREE) { _verifyTree(n);}
      return temp;
    }
    if (b > 1 && (n->left != NULL) && k > _key[n->left->id]) {
      // Left Right case - left then right rotation
      n->left = LeftRotate(n->left);
      auto temp = RightRotate(n);
      if (DEBUG_AVLTREE) { _verifyTree(n);}
      return temp;
    }
    if (b < -1 && (n->right != NULL) && k < _key[n->right->id]) {
      // Right Left case - right then left rotation
      n->right = RightRotate(n->right);
      auto temp = LeftRotate(n);
      if (DEBUG_AVLTREE) { _verifyTree(n);}
      return temp;
    }
    if (DEBUG_AVLTREE) { _verifyTree(n);}
    return n; // return unchanged node
  };
  
  // Find the node with minimum key in subtree rooted at n
  template <typename DataType>
  AVLNode* AVLTree<DataType>::_findMin(AVLNode* n) {
    if (n == NULL) {return NULL;}
    AVLNode* current = n;
    // find the left-most leaf
    while (current->left != NULL)
      current = current->left;
    return current;
  };
  
  // Delete a node with key k (recursive)
  template <typename DataType>
  AVLNode* AVLTree<DataType>::_delete(AVLNode* n, DataType k) {
    // Standard BST delete first
    if (n == NULL) {
      return n;
    }
    if ( k < _key[n->id] ) {
      n->left = _delete(n->left, k);
    } else if( k > _key[n->id] ) {
      n->right = _delete(n->right, k);
    } else { // this is the node to be deleted
      // Node with one child or no child
      if( (n->left == NULL) ||
          (n->right == NULL) ) 
      { 
        AVLNode *child = n->left ? n->left : n->right;
        if (child == NULL) { // no child case
          delete n;
          _size--;
          n = NULL;
        } else {// one child case
          *n = *child; // copy child's data
          if (child == n->left) {n->left = NULL;}
          else {n->right = NULL;}
          delete child;
          _size--;
          child = NULL;
        }
      } else { // node with two children
        // Get the inorder successor (smallest in right subtree)
        AVLNode* temp = _findMin(n->right);
        n->id = temp->id; // copy successor's data
        // Delete the inorder successor
        n->right = _delete(n->right, _key[temp->id]);
      }
    }
  
    if (DEBUG_AVLTREE) {_verifyLessThan(n);}
  
    if (n == NULL) { // tree is empty now
      return n;
    }
    
    // Update height
    n->h = 1 + Max(H(n->left), H(n->right));
    
    // Rebalance the tree
    int b = GetBalanceFactor(n);
    if ( (b>1) && (GetBalanceFactor(n->left) >= 0) ) {
      // Left Left case
      auto temp = RightRotate(n);
      return temp;
    }
    if ( (b>1) && (GetBalanceFactor(n->left) < 0) ) {
      // Left Right case
      n->left = LeftRotate(n->left);
      auto temp = RightRotate(n);
      return temp;
    }
    if ( (b<-1) && (GetBalanceFactor(n->right) <= 0) ) {
      // Right Right case
      auto temp = LeftRotate(n);
      return temp;
    } 
    if ( (b<-1) && (GetBalanceFactor(n->right) > 0) ) {
      // Right Left case
      n->right = RightRotate(n->right);
      auto temp = LeftRotate(n);
      return temp;
    }
    return n; // return unchanged node
  }
  
  // Delete all nodes in subtree rooted at n (recursive)
  template <typename DataType>
  void AVLTree<DataType>::_deleteAll(AVLNode* n) {
    if (n == NULL) {
      return ;
    }
    _deleteAll(n->left);
    _deleteAll(n->right);
    delete n;
    return;
  };
  
  // Verify tree properties (for debugging)
  template <typename DataType>
  void AVLTree<DataType>::_verifyTree(AVLNode* n) {
    if (n == NULL) {return;}
  
    _verifyLessThan(n);
  
    // Check balance factor
    int b = GetBalanceFactor(n);
    if ((b < -1) || (b > 1)) {
      std::cout << "[DEBUG] current node = " << (*n) << std::endl;
      throw std::runtime_error("[ERROR] AVLTree is not balanced!");
    }
  
    // Check left child balance
    if (n->left) {
      int b = GetBalanceFactor(n->left);
      if ((b < -1) || (b > 1)) {
        std::cout << "[DEBUG] n->left node = " << (*n->left) << std::endl;
        throw std::runtime_error("[ERROR] AVLTree is not balanced!");
      }
    }
  
    // Check right child balance
    if (n->right) {
      int b = GetBalanceFactor(n->right);
      if ((b < -1) || (b > 1)) {
        std::cout << "[DEBUG] n->right node = " << (*n->right) << std::endl;
        throw std::runtime_error("[ERROR] AVLTree is not balanced!");
      }
    }
    return;
  };
  
  // Verify BST property (left < current < right)
  template <typename DataType>
  void AVLTree<DataType>::_verifyLessThan(AVLNode* n) {
    if (n == NULL) {return;}
    if ( (n->left && _key[n->id] < _key[n->left->id]) || 
          (n->right && _key[n->id] > _key[n->right->id]) )
    {
      std::cout << " _key[n] = " << _key[n->id] << ", ";
      if (n->left) {std::cout << "_key[n->left] = " << _key[n->left->id] << ", ";}
      if (n->right) {std::cout << "_key[n->right] = " << _key[n->right->id] << ", ";}
      std::cout << std::endl;
      throw std::runtime_error("[ERROR] AVL-tree is not a binary search tree !?");
    }
    return;
  };
  
  // Helper function for template compilation
  template<typename DataType>
  void AVLTreeCompileHelper() {
    DataType a;
    AVLTree<DataType> t;
    t.Add(a);
    t.Size();
    t.Print();
    DataType b;
    t.Find(a);
    t.FindMaxLess(a, &b, false, NULL);
    t.FindMinMore(a, &b, false, NULL);
    t.Delete(a);
    t.Clear();
  };
  
  // Explicit template instantiation for common types
  void AVLTreeInstantiation(){
    AVLTreeCompileHelper<int>();
    AVLTreeCompileHelper<short>();
    AVLTreeCompileHelper<long>();
    AVLTreeCompileHelper<float>();
    AVLTreeCompileHelper<double>();
    AVLTreeCompileHelper< std::vector<int> >();
    AVLTreeCompileHelper< std::vector<short> >();
    AVLTreeCompileHelper< std::vector<long> >();
    AVLTreeCompileHelper< std::vector<float> >();
    AVLTreeCompileHelper< std::vector<double> >();
  }
  
  // Explicit template instantiation
  template class AVLTree<int>;
  template class AVLTree<short>;
  template class AVLTree<long>;
  template class AVLTree<float>;
  template class AVLTree<double>;
  template class AVLTree< std::vector<int> >;
  template class AVLTree< std::vector<short> >;
  template class AVLTree< std::vector<long> >;
  template class AVLTree< std::vector<float> >;
  template class AVLTree< std::vector<double> >;
  
  
} // namespace raplab