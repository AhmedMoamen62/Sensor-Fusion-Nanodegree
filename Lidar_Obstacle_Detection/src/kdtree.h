/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

    template<typename PointT>
    Node(PointT arr, int setId)
    :	id(setId), left(NULL), right(NULL)
    {
        point.push_back(arr.x);
        point.push_back(arr.y);
        point.push_back(arr.z);
    }
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

    template<typename PointT>
    void insertAt(PointT point, int id, Node* &node, uint depth)
    {
        if(node == NULL)
        {
            node = new Node(point,id);
            return;
        }

        if(id == node->id)
            return;

        uint current_depth = depth % 2;
        if(current_depth == 0)
        {
            if(point.x < node->point[0])
                insertAt(point, id, node->left, depth + 1);
            else
                insertAt(point, id, node->right, depth + 1);
        }
        else
        {
            if(point.y < node->point[1])
                insertAt(point, id, node->left, depth + 1);
            else
                insertAt(point, id, node->right, depth + 1);
        }
    }

    template<typename PointT>
    void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root

        insertAt(point, id, this->root, 0);
	}

    template<typename PointT>
    void searchAt(PointT target, float distanceTol, Node* node, uint depth,std::vector<int>& ids)
    {
        if(node == NULL)
            return;

        float DifX = node->point[0] - target.x;
        float DifY = node->point[1] - target.y;

        if(node->point[0] >= target.x - distanceTol && node->point[0] <= target.x + distanceTol && node->point[1] >= target.y - distanceTol && node->point[1] <= target.y + distanceTol)
        {
            if(sqrt(DifX*DifX + DifY*DifY) <= distanceTol)
                ids.push_back(node->id);
        }

        uint current_depth = depth % 2;

        if(current_depth == 0)
        {
            if(target.x - distanceTol < node->point[0])
                searchAt(target,distanceTol,node->left,depth + 1,ids);
            if(target.x + distanceTol > node->point[0])
                searchAt(target,distanceTol,node->right,depth + 1,ids);
        }
        else
        {
            if(target.y - distanceTol < node->point[1])
                searchAt(target,distanceTol,node->left,depth + 1,ids);
            if(target.y + distanceTol > node->point[1])
                searchAt(target,distanceTol,node->right,depth + 1,ids);
        }
    }

	// return a list of point ids in the tree that are within distance of target
    template<typename PointT>
    std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
        searchAt(target,distanceTol,this->root,0,ids);
		return ids;
	}
	

};




