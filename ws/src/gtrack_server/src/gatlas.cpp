#include <iostream>
#include "gtrack_server/gatlas.hpp"

GAtlas::GAtlas() {};

GAtlas::~GAtlas() {};


void GAtlas::insert(int a, int b, const TransformData& TR) {
	if (_gatlas.count(a) == 0) {
		// Add device
		AtlasElement el;
		el.id = a;
		el.relations.insert(std::pair<int, TransformData>(b, TR)); 
		_gatlas.insert(std::pair<int, AtlasElement>(a, el));
	} else {
		_gatlas[a].relations[b] = TR;
	}

	TransformData invTR;
	invTR.rot = TR.rot.inverse();
	invTR.t = invTR.rot * (-TR.t);

	if (_gatlas.count(b) == 0) {
		// Add device
		AtlasElement el;
		el.id = b;
		
		el.relations.insert(std::pair<int, TransformData>(a, invTR)); 
		_gatlas.insert(std::pair<int, AtlasElement>(b, el));
	} else {
		_gatlas[b].relations[a] = invTR;
	}

}

bool GAtlas::getTransform(int orig, int dest, TransformData& tr) {
	bool output = false;

	// Check whether the nodes are in the map
	if (_gatlas.count(orig) == 0 || _gatlas.count(dest) == 0) {
		return output;
	}

	_visited.clear(); 
	if (find_path(orig, dest, tr)) {
		output = true;;
	}
	return output;
}

bool GAtlas::find_path(int s, int e, TransformData& TR) {
	bool out = false;

	if (s == e) {
		TR.rot = Eigen::Quaterniond::Identity();
		TR.t = Eigen::Vector3d::Zero();
		return true;
	}

	// Nodes connected to the current node 's' 
	std::unordered_map<int, TransformData> rels = _gatlas[s].relations;
	// Add the current node to the set of visited nodes
	_visited.insert(s);

	// For each connected node
	for (auto el : rels) {
		int nextId = el.first;
		TransformData locTR = el.second;

		TransformData downstreamTR;

		if (_visited.count(nextId) > 0) 
			continue;

		out = find_path(nextId, e, downstreamTR);
		if (out) {
			/*
			std::cout << "Path: " << nextId << std::endl; 
			std::cout << "      " << locTR.t.transpose() << std::endl; 
			std::cout << "      " << locTR.rot.w() << " " <<
				locTR.rot.vec().transpose() << std::endl;
			*/
			TR.t = locTR.rot * downstreamTR.t + locTR.t;
			TR.rot = locTR.rot * downstreamTR.rot;
			return true;
		}
	}
	return false;
}
