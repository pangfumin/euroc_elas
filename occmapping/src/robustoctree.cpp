/*
 * Copyright (c) 2013-2014, Konstantin Schauwecker
 * 
 * This code is based on the original OctoMap implementation. The original
 * copyright notice and source code license are as follows:
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robustoctree.h"
#include <cmath>
#include <list>
#include <exception>

#define MODEL_Z_ERROR // remove this constant if depth error should not be modeled

namespace occmapping {
	using namespace octomap;
	using namespace std;
	
	// A dummy value we use to represent the root key of a ray
	octomap::OcTreeKey RobustOcTree::ROOT_KEY(0xFFFF, 0xFFFF, 0xFFFF);

	RobustOcTree::StaticMemberInitializer RobustOcTree::ocTreeMemberInit;

	RobustOcTree::RobustOcTree()
		:Base(RobustOcTreeParameters().octreeResolution) {
		initFromParameters();
	}
	
	RobustOcTree::RobustOcTree(const RobustOcTreeParameters& parameters)
		:Base(parameters.octreeResolution), parameters(parameters) {
		initFromParameters();
	}
	
	RobustOcTree::~RobustOcTree() {
		delete lookup;
	}
	
	void RobustOcTree::initFromParameters() {
		// We keep the inverse probabilities too
		probMissIfOccupied = 1.0 - parameters.probHitIfOccupied;
		probMissIfNotOccupied = 1.0 - parameters.probHitIfNotOccupied;
		probMissIfNotVisible = 1.0 - parameters.probHitIfNotVisible;
		
		// Convert minimum and maximum height to octree keys
		OcTreeKey key;
		if(!coordToKeyChecked(0, 0, parameters.minHeight, key))
			minZKey = 0;
		else minZKey = key[2];

		if(!coordToKeyChecked(0, 0, parameters.maxHeight, key))
			maxZKey = 0xFFFF;
		else maxZKey = key[2];
		
		// Generate probability look-up table.
		// Some parameters are hard coded here, but you propability don't want to change them anyway.
		lookup = new ProbabilityLookup(parameters.octreeResolution, parameters.depthErrorResolution, 0.1,
			parameters.maxRange, 0, 1, 0.05, parameters.depthErrorScale);
		
		// Hopefully makes accesses to the hash faster
		voxelUpdateCache.max_load_factor(0.75);
		voxelUpdateCache.rehash(1000);
	}

	void RobustOcTree::insertPointCloud(const octomap::Pointcloud& scan, const octomap::point3d& sensor_origin,
		const octomap::point3d& forwardVec, bool lazy_eval) {
		
		// Do ray casting stuff
		computeUpdate(scan, sensor_origin, forwardVec);
		
		// Calculate a visibility for all voxels that will be updated
		calcVisibilities(sensor_origin);

		// Perform update
		for (OccPrevKeySet::iterator it = voxelUpdateCache.begin(); it != voxelUpdateCache.end(); it++) {
			float visibility = visibilities[it->key];
			if(visibility >= parameters.visibilityClampingMin)
				updateNode(it->key, visibility, it->occupancy, lazy_eval);
		}
	}
	
	void RobustOcTree::calcVisibilities(const octomap::point3d& sensor_origin) {
		// Find the origin key
		OcTreeKey originKey;
		if ( !coordToKeyChecked(sensor_origin, originKey)) {
			OCTOMAP_WARNING_STR("origin coordinates ( " << sensor_origin << ") out of bounds in computeRayKeys");
			return;
		}
		
		visibilities.clear();
		visibilities[ROOT_KEY] = 1.0;
	
		// Create list of keys that need to be processed
		list<const OccPrevKey*> unprocessedKeys(voxelUpdateCache.size());
		list<const OccPrevKey*>::iterator it2 = unprocessedKeys.begin();
		
		for (OccPrevKeySet::iterator it1 = voxelUpdateCache.begin(); it1 != voxelUpdateCache.end(); it1++, it2++)
			(*it2) = &(*it1);
	
		// Perform processing until there are no more keys left
		while(unprocessedKeys.size() > 0) {
			for (list<const OccPrevKey*>::iterator it = unprocessedKeys.begin(); it != unprocessedKeys.end();) {
				KeyVisMap::iterator visIter = visibilities.find((*it)->prevKey);
				if(visIter != visibilities.end()) {
					// Find occupancies for the three neighbor nodes bordering
					// the visible faces
					octomap::OcTreeKey currKey = (*it)->key;
					
					int step[3] = {
						currKey[0] > originKey[0] ? 1 : -1,
						currKey[1] > originKey[1] ? 1 : -1,
						currKey[2] > originKey[2] ? 1 : -1
					};
					OcTreeKey neighborKeys[3] =  {
						OcTreeKey(currKey[0] - step[0], currKey[1], currKey[2]),
						OcTreeKey(currKey[0], currKey[1] - step[1], currKey[2]),
						OcTreeKey(currKey[0], currKey[1], currKey[2] - step[2])
					};
					
					float occupancies[3] = {0.5F, 0.5F, 0.5F};

					for(unsigned int i = 0; i<3; i++) {
						RobustOcTreeNode* neighborNode = search(neighborKeys[i]);
						if(neighborNode != NULL) {
							occupancies[i] = neighborNode->getOccupancy();
							if(occupancies[i] <= parameters.clampingThresholdMin)
								occupancies[i] = 0;
						}
					}
					
					// Get the probabilities that the current voxel is occluded by its neighbors
					float occlusionProb = min(occupancies[0], min(occupancies[1], occupancies[2]));
					float visibility = visIter->second * (parameters.probVisibleIfOccluded * occlusionProb
						+ parameters.probVisibleIfNotOccluded * (1.0F-occlusionProb));

					// clamp visibility
					if(visibility > parameters.visibilityClampingMax)
						visibility = 1.0F;
				
					visibilities[(*it)->key] = visibility;
					unprocessedKeys.erase(it++);
				} else ++it;
			}
		}
	}
	
	void RobustOcTree::insertPointCloud(const Pointcloud& pc, const point3d& sensor_origin, const pose6d& frame_origin, bool lazy_eval) {
		// performs transformation to data and sensor origin first
		Pointcloud transformed_scan (pc);
		transformed_scan.transform(frame_origin);
		point3d transformed_sensor_origin = frame_origin.transform(sensor_origin);
		point3d forwardVec = frame_origin.transform(sensor_origin + point3d(0, 1.0, 0.0)) - transformed_sensor_origin;
		
		// Process transformed point cloud
		insertPointCloud(transformed_scan, transformed_sensor_origin, forwardVec, lazy_eval);
	}
	
	void RobustOcTree::computeUpdate(const Pointcloud& scan, const octomap::point3d& origin, const octomap::point3d& forwardVec) {
		voxelUpdateCache.clear();
		prevKeyray.reset();
		
		for (int i = 0; i < (int)scan.size(); ++i) {
			// Compute a new ray
			const point3d& p = scan[i];
			computeRayKeys(origin, p, forwardVec);
			
			// Insert cells into the update cache if they haven't been
			// part of a previous ray, or if their occupancy is higher than
			// for any previous ray.
			
			// First perform lookup against previous ray
			OccPrevKeyRay::iterator currIt = keyray.begin();
			OccPrevKeyRay::iterator currEnd = prevKeyray.size() > keyray.size() ? keyray.end() : keyray.begin() + prevKeyray.size();
			
			for(OccPrevKeyRay::iterator prevIt = prevKeyray.begin(); currIt != currEnd; currIt++, prevIt++) {
				if(currIt->key == prevIt->key) {
					// Cell already exists in previous ray
					if(currIt->occupancy > prevIt->occupancy) {
						OccPrevKeySet::iterator updateIter = voxelUpdateCache.find(*currIt);
						OccPrevKeySet::iterator insertHint = voxelUpdateCache.erase(updateIter);
						voxelUpdateCache.insert(insertHint, *currIt);
					} else {
						currIt->occupancy = prevIt->occupancy; // Keep best occupancy for next ray
					}
				} else {
					// Perform full lookup against hash map
					OccPrevKeySet::iterator updateIter = voxelUpdateCache.find(*currIt);
					if(updateIter == voxelUpdateCache.end()) {
						voxelUpdateCache.insert(*currIt);
					} else if(currIt->occupancy > updateIter->occupancy) {
						OccPrevKeySet::iterator insertHint = voxelUpdateCache.erase(updateIter);					
						voxelUpdateCache.insert(insertHint, *currIt);
					} else {
						currIt->occupancy = updateIter->occupancy; // Keep best occupancy for next ray
					}
				}
			}
			
			// Lookup remaining cells against hash map
			for(; currIt != keyray.end(); currIt++) {
				OccPrevKeySet::iterator updateIter = voxelUpdateCache.find(*currIt);
				if(updateIter == voxelUpdateCache.end()) {
					voxelUpdateCache.insert(*currIt);
				} else if(currIt->occupancy > updateIter->occupancy) {
					OccPrevKeySet::iterator insertHint = voxelUpdateCache.erase(updateIter);
					voxelUpdateCache.insert(insertHint, *currIt);
				} else {
					currIt->occupancy = updateIter->occupancy; // Keep best occupancy for next ray
				}
			}
			
			keyray.swap(prevKeyray);
		}
	}
	
	void RobustOcTree::computeRayKeys(const point3d& origin, const point3d& end, const octomap::point3d& forwardVec) {
		// see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
		// basically: DDA in 3D
		
		keyray.reset();
		
		point3d direction = (end - origin);
		double length = direction.norm();
		direction /= length; // normalize vector

		OcTreeKey key_origin, key_end;
		if ( !coordToKeyChecked(origin, key_origin)) {
			OCTOMAP_WARNING_STR("origin coordinates ( " << origin << " -> " << end << ") out of bounds in computeRayKeys");
			return;
		}
		if(!coordToKeyChecked(end, key_end) && !coordToKeyChecked(origin + direction*(2.0*parameters.maxRange), key_end)) {
			OCTOMAP_WARNING_STR("end coordinates ( " << origin << " -> " << end << ") out of bounds in computeRayKeys");
			return;
		}
		
		// Initialization phase -------------------------------------------------------

		int	   step[3];
		double tMax[3];
		double tDelta[3];

		OcTreeKey prev_key = ROOT_KEY;
		OcTreeKey current_key = key_origin; 

		for(unsigned int i=0; i < 3; ++i) {
			// compute step direction
			if (direction(i) > 0.0) step[i] =	1;
			else if (direction(i) < 0.0)	 step[i] = -1;
			else step[i] = 0;

			// compute tMax, tDelta
			if (step[i] != 0) {
				// corner point of voxel (in direction of ray)
				double voxelBorder = keyToCoord(current_key[i]);
				voxelBorder += step[i] * resolution * 0.5;

				tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
				tDelta[i] = this->resolution / fabs( direction(i) );
			}
			else {
				tMax[i] =	std::numeric_limits<double>::max( );
				tDelta[i] = std::numeric_limits<double>::max( );
			}
		}

		// for speedup:
		point3d origin_scaled = origin;
		origin_scaled /= resolution;	
		
		double length_to_key = (keyToCoord(key_end) - origin).norm();
		double length_scaled = (min(length_to_key, parameters.maxRange)) / resolution;
		length_scaled = length_scaled*length_scaled;
		double maxrange_scaled = parameters.maxRange / resolution;
		maxrange_scaled *= maxrange_scaled;
		
		// Conversion factor from length to depth (length of one z-step)
		double lengthToDepth = forwardVec.x() * direction.x() + forwardVec.y() * direction.y() + forwardVec.z() * forwardVec.z();
		
		double depth = length / lengthToDepth;
		double probApproxMaxDistScaled, probApproxMaxDist;
		vector<float>* lookupEntry = lookup->lookupEntry(depth);
		if(lookupEntry == NULL) {
			probApproxMaxDist = std::numeric_limits<double>::max();
			probApproxMaxDistScaled = maxrange_scaled;
		} else {
			probApproxMaxDist = lengthToDepth*(depth - (lookupEntry->size()/2.0)*parameters.depthErrorResolution);
			probApproxMaxDistScaled = probApproxMaxDist / resolution;
			probApproxMaxDistScaled = min(probApproxMaxDistScaled*probApproxMaxDistScaled, maxrange_scaled);
		}
		double treeMax05 = tree_max_val - 0.5F;
		// Incremental phase	---------------------------------------------------------

		unsigned int dim = 0;
		double squareDistFromOrigVec[3] = {
			(current_key[0] - treeMax05 - origin_scaled(0))*(current_key[0] - treeMax05 - origin_scaled(0)),
			(current_key[1] - treeMax05 - origin_scaled(1))*(current_key[1] - treeMax05 - origin_scaled(1)),
			(current_key[2] - treeMax05 - origin_scaled(2))*(current_key[2] - treeMax05 - origin_scaled(2))};
		
		while(true) {
			// Calculate distance from origin
			double squareDistFromOrig = squareDistFromOrigVec[0] + squareDistFromOrigVec[1] + squareDistFromOrigVec[2];
			
#ifdef MODEL_Z_ERROR
			if(squareDistFromOrig < probApproxMaxDistScaled) {
				// Use approximate probabilities
				keyray.addOccPrevKey(0.0, current_key, prev_key);
			} else if(squareDistFromOrig >= maxrange_scaled) {
				// The point is too far away. Lets stop.
				break;
			} else {
				// Detailed calculation
				int index = int((sqrt(squareDistFromOrig)*resolution - probApproxMaxDist)/
					(parameters.depthErrorResolution*lengthToDepth) + 0.5);
				
				double occupancyFactor = 1.0;
				bool done = false;
				if(index >= (int)lookupEntry->size()) {
					done = true;
					// Continue to make sure we integrate one full hit
				} else {
					occupancyFactor = (*lookupEntry)[index];
				}
				
				keyray.addOccPrevKey(occupancyFactor, current_key, prev_key);
				
				if(done)
					break;
			}
#else
			// reached endpoint?
			if (current_key == key_end) {
                keyray.addOccPrevKey(1.0, current_key, prev_key);
                break;
			} else if(squareDistFromOrig >= length_scaled) {
				// We missed it :-(
				if(length_to_key < parameters.maxRange)
					keyray.addOccPrevKey(1.0, key_end, prev_key);
				break;
			} else {
				keyray.addOccPrevKey(0.0, current_key, prev_key);
			}
#endif
			
			// find minimum tMax:
			if (tMax[0] < tMax[1]){
				if (tMax[0] < tMax[2]) dim = 0;
				else dim = 2;
			}
			else {
				if (tMax[1] < tMax[2]) dim = 1;
				else dim = 2;
			}

			// advance in direction "dim"
			prev_key = current_key;
			current_key[dim] += step[dim];
			tMax[dim] += tDelta[dim];
			squareDistFromOrigVec[dim] = current_key[dim] - treeMax05 - origin_scaled(dim);
			squareDistFromOrigVec[dim]*= squareDistFromOrigVec[dim];
			
			assert (current_key[dim] < 2*this->tree_max_val);
			
			if(current_key[2] < minZKey || current_key[2] > maxZKey)
				break; // Exceeded min or max height
				
			assert ( keyray.size() < keyray.sizeMax() - 1);
		}
	}
	
	RobustOcTreeNode* RobustOcTree::updateNode(const OcTreeKey& key, float visibility, float occupancy, bool lazy_eval) {
		// early abort (no change will happen).
		// may cause an overhead in some configuration, but more often helps
		RobustOcTreeNode* leaf = this->search(key);
		// no change: node already at minimum threshold
		if (leaf != NULL && occupancy == 0.0F && leaf->getOccupancy() <= parameters.clampingThresholdMin)
			return leaf;

		bool createdRoot = false;
		if (this->root == NULL){
			this->root = new RobustOcTreeNode();
			this->tree_size++;
			createdRoot = true;
		}

		return updateNodeRecurs(this->root, visibility, createdRoot, key, 0, occupancy, lazy_eval);
	}
	
	RobustOcTreeNode* RobustOcTree::updateNodeRecurs(RobustOcTreeNode* node, float visibility, bool node_just_created, const OcTreeKey& key,
		unsigned int depth, float occupancy, bool lazy_eval) {
		// Differences to OccupancyOcTreeBase implementation: only cosmetic
		
		unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
		bool created_node = false;

		assert(node);

		// follow down to last level
		if (depth < this->tree_depth) {
			if (!node->childExists(pos)) {
				// child does not exist, but maybe it's a pruned node?
				if ((!node->hasChildren()) && !node_just_created ) {
					// current node does not have children AND it is not a new node 
					// -> expand pruned node
					node->expandNode();
					this->tree_size+=8;
					this->size_changed = true;
				}
				else {
					// not a pruned node, create requested child
					node->createChild(pos);
					this->tree_size++;
					this->size_changed = true;
					created_node = true;
				}
			}

			if (lazy_eval)
				return updateNodeRecurs(node->getChild(pos), visibility, created_node, key, depth+1, occupancy, lazy_eval);
			else {
				RobustOcTreeNode* retval = updateNodeRecurs(node->getChild(pos), visibility, created_node, key, depth+1, occupancy, lazy_eval);
				// prune node if possible, otherwise set own probability
				// note: combining both did not lead to a speedup!
				if (node->pruneNode())
					this->tree_size -= 8;
				else
					node->updateOccupancyChildren();

				return retval;
			}
		}

		// at last level, update node, end of recursion
		else {
			updateNodeOccupancy(node, visibility, occupancy);
			return node;
		}
	}
	
	void RobustOcTree::updateNodeOccupancy(RobustOcTreeNode* occupancyNode, float probVisible, float occupancy) {
		float probOccupied = occupancyNode->getOccupancy();
		
		// Decide which update procedure to perform
		if(occupancy == 0.0F) {
			// Perform update for certainly free voxels
			float probMiss = calcOccupiedProbability(probOccupied, probVisible, probMissIfOccupied, probMissIfNotOccupied, probMissIfNotVisible);
			occupancyNode->setOccupancy(probMiss > parameters.clampingThresholdMin ? probMiss : parameters.clampingThresholdMin);
		} else {
			float probHit = calcOccupiedProbability(probOccupied, probVisible,
				parameters.probHitIfOccupied, parameters.probHitIfNotOccupied, parameters.probHitIfNotVisible);
			if(occupancy == 1.0F) {
				// Perform update for certainly occupied voxels
				occupancyNode->setOccupancy(probHit < parameters.clampingThresholdMax ? probHit : parameters.clampingThresholdMax);
			} else {
				// We are not certain, but we know the occupancy probability.
				// Lets interpolate between the update for free and occupied voxels.
				float probMiss = calcOccupiedProbability(probOccupied, probVisible, probMissIfOccupied, probMissIfNotOccupied, parameters.probHitIfNotVisible);
				occupancyNode->setOccupancy(occupancy*probHit + (1.0F-occupancy)*probMiss);
				
				// Perform probability clamping
				if (occupancyNode->getOccupancy() < parameters.clampingThresholdMin)
					occupancyNode->setOccupancy(parameters.clampingThresholdMin);
				else if (occupancyNode->getOccupancy() > parameters.clampingThresholdMax)
					occupancyNode->setOccupancy(parameters.clampingThresholdMax);
			}
		}
	}
	
	float RobustOcTree::calcOccupiedProbability(float probOccupied, float probVisible, float probMeasIfOccupied, float probMeasIfNotOccupied,
		float probMeasNotVisible) {
		// This is our update equation. See our paper for a derivation of this formula (Eq. 4 - 6).
		return (probMeasIfOccupied*probVisible*probOccupied + probMeasNotVisible*(1.0F-probVisible)*probOccupied) /
			(probMeasIfOccupied*probVisible*probOccupied + probMeasIfNotOccupied*probVisible*(1.0F - probOccupied) +
				probMeasNotVisible*(1.0F-probVisible));
	}
	
	void RobustOcTree::useBBXLimit(bool enable) {
		struct ExceptionType: std::exception {
			char const* what() const throw() {
				return "Not supported!";
			}
		};
		
		throw ExceptionType();
	}
	
	void RobustOcTree::enableChangeDetection (bool enable) {
		struct ExceptionType: std::exception {
			char const* what() const throw() {
				return "Not supported!";
			}
		};
		
		throw ExceptionType();
	}
}
