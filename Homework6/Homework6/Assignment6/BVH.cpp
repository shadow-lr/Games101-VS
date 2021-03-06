#include <algorithm>
#include <cassert>
#include <map>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    switch (splitMethod)
    {
    case BVHAccel::SplitMethod::NAIVE:
        printf("\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n", hrs, mins, secs);
        break;
    case BVHAccel::SplitMethod::SAH:
        printf("\rSAH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n", hrs, mins, secs);
        break;
    default:
        break;
    }
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    std::vector<Object*> leftshapes;
    std::vector<Object*> rightshapes;

    for (int i = 0; i < objects.size(); ++i)
    {
        bounds = Union(bounds, objects[i]->getBounds());
    }
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
        {
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        }

		switch (splitMethod)
		{
		case SplitMethod::NAIVE: {
            BuildBVH(node, objects, centroidBounds, leftshapes, rightshapes);
			break;
		}
		case SplitMethod::SAH: {
            BuildSAH(node, objects, centroidBounds, leftshapes, rightshapes);
			break;
		}
		default:
            break;
        }
    }

    assert(objects.size() == (leftshapes.size() + rightshapes.size()));

    node->left = recursiveBuild(leftshapes);
    node->right = recursiveBuild(rightshapes);

    node->bounds = Union(node->left->bounds, node->right->bounds);

    return node;
}

void BVHAccel::BuildBVH(BVHBuildNode*& node, std::vector<Object*>& objects, Bounds3& centroidBounds, std::vector<Object*>& leftshapes, std::vector<Object*>& rightshapes)
{
	for (int i = 0; i < objects.size(); ++i)
	{
		// ????????????
		centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
	}
	// ?????? ?????????????????? x : 0 y : 1 z : 2
	// ????????????????????????????, x?????? ????x??????????
	int dim = centroidBounds.maxExtent();
	switch (dim) {
	case 0:
		std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
			return f1->getBounds().Centroid().x <
				f2->getBounds().Centroid().x;
		});
		break;
	case 1:
		std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
			return f1->getBounds().Centroid().y <
				f2->getBounds().Centroid().y;
		});
		break;
	case 2:
		std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
			return f1->getBounds().Centroid().z <
				f2->getBounds().Centroid().z;
		});
		break;
	}

	auto beginning = objects.begin();
	auto middling = objects.begin() + (objects.size() / 2);
	auto ending = objects.end();

	// ????left mid
	leftshapes = std::vector<Object*>(beginning, middling);
	// ????mid right
	rightshapes = std::vector<Object*>(middling, ending);
}

void BVHAccel::BuildSAH(BVHBuildNode*& node, std::vector<Object*>& objects, Bounds3& centroidBounds, std::vector<Object*>& leftshapes, std::vector<Object*>& rightshapes)
{
    Bounds3 nBounds;
    for (int i = 0; i < objects.size(); ++i)
    {
        nBounds = Union(nBounds, objects[i]->getBounds());
    }

    float nArea = centroidBounds.SurfaceArea();

    int minCostCoor = 0;
    int minCostIndex = 0;

    float minCost = std::numeric_limits<float>::infinity();

    // <i, map<j bid>>
    // i : ????????
    // j : ????????
    // bid : ??????????????
    std::map<int, std::map<int, int>> indexMap;

    int bucketCount = 12;

    // ??????????????build
    for (int i = 0; i < 3; ++i)
    {

        // ????????????
        std::vector<Bounds3> boundsBuckets;
        std::vector<int> countBucket;

        for (int i = 0; i < bucketCount; ++i)
        {
            boundsBuckets.push_back(Bounds3());
            countBucket.push_back(0);
        }

        // object index mapping ?? index
        std::map<int, int> objMap;

        // traversal
        for (int j = 0; j < objects.size(); ++j)
        {
            // ??????0~12
            // bid ?????? ??????????object??Bounds??centroidBounds.pMin??????????
            int bid = bucketCount * (centroidBounds.Offset(objects[j]->getBounds().Centroid()))[i];
            if (bid >= bucketCount)
            {
                bid = bucketCount - 1;
            }

            boundsBuckets[bid] = Union(boundsBuckets[bid], objects[j]->getBounds().Centroid());
            countBucket[bid] += 1;
            objMap.insert(std::make_pair(j, bid));
        }

        indexMap.insert(std::make_pair(i, objMap));

        // ??????????????????????????????????????????????????????????????????????????????????????????
        // traversal from 1 to boundsBuckets.size() - 1 j??[1, boundsBuckets.size() - 1]
        for (int j = 1; j < boundsBuckets.size(); ++j)
        {
            Bounds3 A, B;
            int countA = 0, countB = 0;
            
            // A??????j????
            for (int k = 0; k < j; ++k)
            {
                A = Union(A, boundsBuckets[k]);
                countA += countBucket[k];
            }

            // B??????????
            for (int k = 0; k < boundsBuckets.size(); ++k)
            {
                B = Union(B, boundsBuckets[k]);
                countB += countBucket[k];
            }
            
            // ????????A??????j???? ?? ????????????
            float cost = 1 + (countA * A.SurfaceArea() + countB * B.SurfaceArea()) / nArea;

            if (cost < minCost)
            {
                minCost = cost;
                // A??j??????????????????
                minCostIndex = j;
                // ???????????????????????? (x, y, z)
                minCostCoor = i;
            }
        }

        for (int i = 0; i < objects.size(); ++i)
        {
            // bid < minCostIndex
            if (indexMap[minCostCoor][i] < minCostIndex)
            {
                leftshapes.push_back(objects[i]);
            }
            else
            {
                rightshapes.push_back(objects[i]);
            }
        }
    }
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection isect;

    if (node == nullptr || !node->bounds.IntersectP(ray, ray.direction_inv, { 0, 0, 0 }))
    {
        return isect;
    }
    
    if (node->left == nullptr && node->right == nullptr)
    {
        isect = node->object->getIntersection(ray);
        return isect;
    }

    Intersection leftHit, rightHit;

    leftHit = getIntersection(node->left, ray);
    rightHit = getIntersection(node->right, ray);

    isect = leftHit.distance <= rightHit.distance ? leftHit : rightHit;

    return isect;
}