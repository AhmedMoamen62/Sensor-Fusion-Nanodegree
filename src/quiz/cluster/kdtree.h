/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

    void insertAt(std::vector<float> point, int id, Node* &node, uint depth)
    {
        if(node == NULL)
        {
            node = new Node(point,id);
            return;
        }

        if(id == node->id)
            return;

        uint current_depth = depth % 2;
        if(point[current_depth] < node->point[current_depth])
            insertAt(point, id, node->left, depth + 1);
        else
            insertAt(point, id, node->right, depth + 1);
    }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root

        insertAt(point, id, this->root, 0);
	}

    void searchAt(std::vector<float> target, float distanceTol, Node* node, uint depth,std::vector<int>& ids)
    {
        if(node == NULL)
            return;

        float DifX = node->point[0] - target[0];
        float DifY = node->point[1] - target[1];

        if(node->point[0] >= target[0] - distanceTol && node->point[0] <= target[0] + distanceTol && node->point[1] >= target[1] - distanceTol && node->point[1] <= target[1] + distanceTol)
        {
            if(sqrt(DifX*DifX + DifY*DifY) <= distanceTol)
                ids.push_back(node->id);
        }

        uint current_depth = depth % 2;

        if(target[current_depth] - distanceTol < node->point[current_depth])
            searchAt(target,distanceTol,node->left,depth + 1,ids);
        if(target[current_depth] + distanceTol > node->point[current_depth])
            searchAt(target,distanceTol,node->right,depth + 1,ids);
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        searchAt(target,distanceTol,this->root,0,ids);
		return ids;
	}
	

};




