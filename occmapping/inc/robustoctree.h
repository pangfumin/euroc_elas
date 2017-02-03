/*
 * Copyright (c) 2013-2014, Konstantin Schauwecker
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

#ifndef ROBUSTOCTREE_H
#define ROBUSTOCTREE_H

#include <octomap/OcTreeBase.h>
#include <octomap/OcTree.h>
#include <tr1/unordered_map>
#include "robustoctreenode.h"
#include "robustoctreeparameters.h"
#include "occprevkeyray.h"
#include "probabilitylookup.h"

namespace occmapping {
	// A more noise robust occupancy octree.
	// Not supported features: Bounding box, change notification
	class RobustOcTree: public octomap::OcTreeBase<RobustOcTreeNode> {
	public:
		RobustOcTree();
		RobustOcTree(const RobustOcTreeParameters& parameters);
		
		virtual ~RobustOcTree();
		
		// Methods for processing scans
		virtual void insertPointCloud(const octomap::Pointcloud& scan, const octomap::point3d& sensor_origin,
			const octomap::point3d& forwardVec, bool lazy_eval = false);
			
		virtual void insertPointCloud(const octomap::Pointcloud& pc, const octomap::point3d& sensor_origin,
			const octomap::pose6d& frame_origin, bool lazy_eval = false);

		// virtual constructor: creates a new object of same type
		virtual RobustOcTree* create() const {
			return new RobustOcTree(parameters);
		}
		
		std::string getTreeType() const {
			return "OcTree"; // Ensures compatibility with OcTree
		}

		// Performs thresholding of the occupancy probability for the given node
		inline bool isNodeOccupied(const RobustOcTreeNode& node) const {
			return node.getOccupancy() > parameters.occupancyProbThreshold;
		}

		inline bool isNodeOccupied(const RobustOcTreeNode* node) const {
			return isNodeOccupied(*node);
		}
		
		// Do not call! Not supported!
		void useBBXLimit(bool enable);
		void enableChangeDetection (bool enable);
	
	private:
		typedef octomap::OcTreeBase<RobustOcTreeNode> Base;
		typedef std::tr1::unordered_map<octomap::OcTreeKey, float, octomap::OcTreeKey::KeyHash> KeyVisMap;
		static octomap::OcTreeKey ROOT_KEY;
		
		RobustOcTreeParameters parameters;
		float probMissIfOccupied, probMissIfNotOccupied;
		float probMissIfNotVisible;
		OccPrevKeyRay keyray, prevKeyray;
		OccPrevKeySet voxelUpdateCache;
		KeyVisMap visibilities;
		unsigned short int maxZKey, minZKey;
		ProbabilityLookup* lookup;
							
		/**
		 * Static member object which ensures that this OcTree's prototype
		 * ends up in the classIDMapping only once
		 */
		class StaticMemberInitializer{
		public:
			StaticMemberInitializer() {
				RobustOcTree* tree = new RobustOcTree();
				octomap::AbstractOcTree::registerTreeType(tree);
			}
		};
		
		// to ensure static initialization (only once)
		static StaticMemberInitializer ocTreeMemberInit;
		
		// Performs initialization using the parameters attribute
		void initFromParameters();
		
		// Casts all rays and fills the update cache
		void computeUpdate(const octomap::Pointcloud& scan, const octomap::point3d& origin, const octomap::point3d& forwardVec);
		
		// Casts a single ray and stores it in keyray
		void computeRayKeys(const octomap::point3d& origin, const octomap::point3d& end, const octomap::point3d& forwardVec);
		
		// Manipulate occupancy probability of node directly
		RobustOcTreeNode* updateNodeRecurs(RobustOcTreeNode* node, float visibility, bool node_just_created, const octomap::OcTreeKey& key,
			unsigned int depth, float occupancy, bool lazy_eval);
		RobustOcTreeNode* updateNode(const octomap::OcTreeKey& key, float visibility, float occupancy, bool lazy_eval);
		__always_inline void updateNodeOccupancy(RobustOcTreeNode* occupancyNode, float occupancy, float measProbIfNotOcc);
		
		// Calculates the new occupancy probability
		__always_inline float calcOccupiedProbability(float probOccupied, float probVisible, float probMeasIfOccupied,
			float probMeasIfNotOccupied, float probMeasNotVisible);
		
		// Computes a visibility value for all nodes in voxelUpdateCache
		void calcVisibilities(const octomap::point3d& sensor_origin);
	};
}
#endif
