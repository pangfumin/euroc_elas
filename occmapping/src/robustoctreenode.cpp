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

#include "robustoctreenode.h"

namespace occmapping {
	using namespace std;
	using namespace octomap;

	ostream& RobustOcTreeNode::writeValue(ostream &s) const {	
		// 1 bit for each children; 0: empty, 1: allocated
		std::bitset<8> children;

		for (unsigned int i=0; i<8; i++) {
		  if (childExists(i))
			children[i] = 1;
		  else
			children[i] = 0;
		}

		char children_char = (char) children.to_ulong();
		
		// We convert to logodds to be compatible to OctoMap
		float logVal = logodds(value);
		s.write((const char*) &logVal, sizeof(logVal));
		s.write((char*)&children_char, sizeof(char));

		// write children's children
		for (unsigned int i=0; i<8; i++) {
		  if (children[i] == 1) {
			((RobustOcTreeNode*)this->getChild(i))->writeValue(s);
		  }
		}
		return s;
	}
	
	istream& RobustOcTreeNode::readValue(istream &s) {
		char children_char;

		// read data:
		float logVal;
		s.read((char*) &logVal, sizeof(logVal));
		
		// We convert from logodds to be compatible to OctoMap
		value = probability(logVal);
		
		s.read((char*)&children_char, sizeof(char));
		std::bitset<8> children ((unsigned long long) children_char);

		for (unsigned int i=0; i<8; i++) {
			if (children[i] == 1){
				createChild(i);
				getChild(i)->readValue(s);
			}
		}
		return s;
	}
}
