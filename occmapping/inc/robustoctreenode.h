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

#ifndef ROBUSTOCTREENODE_H
#define ROBUSTOCTREENODE_H

#include <octomap/OcTree.h>

namespace occmapping {
	// An octree node that counts its observations and can be pruned if
	// not observed often enough
	class RobustOcTreeNode : public octomap::OcTreeDataNode<float> {		
	public:
		RobustOcTreeNode() : OcTreeDataNode<float>(0.5) {}
		RobustOcTreeNode(const RobustOcTreeNode& rhs) : OcTreeDataNode<float>(rhs) {}

		// children
		inline RobustOcTreeNode* getChild(unsigned int i) {
			return static_cast<RobustOcTreeNode*> (OcTreeDataNode<float>::getChild(i));
		}
		inline const RobustOcTreeNode* getChild(unsigned int i) const {
			return static_cast<const RobustOcTreeNode*> (OcTreeDataNode<float>::getChild(i));
		}

		bool createChild(unsigned int i) {
			if (children == NULL) allocChildren();
			children[i] = new RobustOcTreeNode();
			return true;
		}
		
		// Returns the occupancy probability
		inline float getOccupancy() const { return value; }
		
		// Sets the occupancy probability
		inline void setOccupancy(float p) { value = p; }
		
		// Methods for compatibility with octomap
		inline void setLogOdds(float l) {value = octomap::probability(l);}
		inline float getLogOdds() const {return octomap::logodds(value);}
		
		// Returns mean of all children's occupancy probabilities, in log odds
		double getMeanChildProbability() const{
			int ctr = 0;
			double sum = 0;
			for (unsigned int i=0; i<8; i++)
				if (childExists(i)) {
					sum += getChild(i)->getOccupancy();
					ctr++;
				}
			if(ctr)
				return sum/ctr;
			else return 0;
		}
		
		// Returns maximum of children's occupancy probabilities
		double getMaxChildProbability() const {
			float max = 0;
			for (unsigned int i=0; i<8; i++) {
				if (childExists(i)) {
					float p = getChild(i)->getOccupancy();
					if(p > max)
						max = p;
				}
			}
					
			return max;
		}
		
		// Update this node's occupancy according to its children's maximum occupancy
		inline void updateOccupancyChildren() {
			this->setOccupancy(this->getMaxChildProbability()); // conservative
		}
		
		// Shadow base class implementations and simulate logodds
		std::ostream& writeValue(std::ostream &s) const;
		std::istream& readValue(std::istream &s);
	};
}

#endif
