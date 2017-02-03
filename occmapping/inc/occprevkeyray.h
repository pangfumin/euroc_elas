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

#ifndef OCCPREVKEYRAY_H
#define OCCPREVKEYRAY_H

#include <vector>
#include <algorithm>
#include <octomap/OcTreeKey.h>

namespace occmapping {
	// Struct storing occupancy and previous key for a given ray key
	struct OccPrevKey {
		float occupancy;
		octomap::OcTreeKey key;
		octomap::OcTreeKey prevKey;

		OccPrevKey(): occupancy(0), key(), prevKey() {
		}
		
		OccPrevKey(float occupancy, const octomap::OcTreeKey& key, const octomap::OcTreeKey& prevKey)
			:occupancy(occupancy), key(key), prevKey(prevKey) {
		}
		
		// The original hashing function did not use a prime number for key.k[1],
		// so I thing that this one is better
		struct KeyHash{
			size_t operator()(const OccPrevKey& val) const{
				// Prime number based hash
				return val.key.k[0] + 1543*val.key.k[1] + 3145739*val.key.k[2];
			}
		};
		
		struct KeyEqual{
			bool operator()(const OccPrevKey& val1, const OccPrevKey& val2) const{
				return val1.key == val2.key;
			}
		};
	};

	// A ray of OccPrevKeys
	class OccPrevKeyRay {
		public:
			OccPrevKeyRay () {
				ray.resize(100000);
				reset();
			}
			void reset() {
				end_of_ray = begin();
			}
			
			void addOccPrevKey(float occupancy, const octomap::OcTreeKey& key, const octomap::OcTreeKey& prevKey) {
				assert(end_of_ray != ray.end());
				end_of_ray->occupancy = occupancy;
				end_of_ray->key = key;
				end_of_ray->prevKey = prevKey;
				end_of_ray++;
			}

			unsigned int size() const { return end_of_ray - ray.begin(); }
			unsigned int sizeMax() const { return ray.size(); }

			typedef std::vector<OccPrevKey>::iterator iterator;
			typedef std::vector<OccPrevKey>::const_iterator const_iterator;
			typedef std::vector<OccPrevKey>::reverse_iterator reverse_iterator;
			
			iterator begin() { return ray.begin(); }
			iterator end() { return end_of_ray; }
			const_iterator begin() const { return ray.begin(); }
			const_iterator end() const	 { return end_of_ray; }

			reverse_iterator rbegin() { return (reverse_iterator) end_of_ray; }
			reverse_iterator rend() { return ray.rend(); }
			
			OccPrevKey& front() {return ray[0];}
			OccPrevKey& back() {return *(end_of_ray-1);}
			
			void swap(OccPrevKeyRay& other) {
				ray.swap(other.ray);
				std::swap(end_of_ray, other.end_of_ray);
			}
			
		private:
			std::vector<OccPrevKey> ray;
			std::vector<OccPrevKey>::iterator end_of_ray;
	};

	typedef std::tr1::unordered_set<OccPrevKey, OccPrevKey::KeyHash, OccPrevKey::KeyEqual> OccPrevKeySet;
}
#endif
