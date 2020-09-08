/**
 * \file gatlas.cpp
 *
 */
#ifdef GATLAS_DEBUG
#include <iostream>
#endif

#include "gatlas/gatlas.hpp"
#include "gtrack_server/rpc_data.hpp"

using namespace gatlas;

GAtlas::GAtlas() {
}

GAtlas::GAtlas(std::string& ip, int port) {
}

GAtlas::~GAtlas() {
}

int GAtlas::update_target_data(
		int id,
		const Eigen::Vector3d& p,
		const Eigen::Vector3d& v,
		uint64_t timestamp) {
	int out = -1;
	
	_targetdata_mx.lock();
	if (_targetdata.count(id) > 0) { // Update
		out = 0;
		_targetdata[id].pos = p + 0.8 * (_targetdata[id].pos - p);
		_targetdata[id].vel = v + 0.8 * (_targetdata[id].vel - v);
		_targetdata[id].timestamp = timestamp;
	} else { // Create
		out = 1;
		TargetData ptg;
		ptg.pos = p;
		ptg.vel = v;
		ptg.Nobservers = 1;
		ptg.timestamp = timestamp;
		_targetdata.insert(std::pair<int, TargetData>(id, ptg));
	}
	_targetdata_mx.unlock();

	return out;
}

bool GAtlas::get_target_data(
		int id,
		Eigen::Vector3d& p,
		Eigen::Vector3d& v,
		uint64_t* timestamp) {
	bool out = false;

	_targetdata_mx.lock();
	if (_targetdata.count(id) > 0) { // In memory
		out = true;
		p = _targetdata[id].pos;
		v = _targetdata[id].vel;
		*timestamp = _targetdata[id].timestamp;
		_targetdata_mx.unlock();
	} 

	return out;
}


void GAtlas::setTransform(int a, int b,
		const TransformData& TR) { // Insert the direct transformation

	_atlasdata_mx.lock();

	if (_gatlas.count(a) == 0) {
		GAtlasElement el;
		el.id = a;
		el.relations.insert(std::pair<int, TransformData>(b, TR)); 
		_gatlas.insert(std::pair<int, GAtlasElement>(a, el));
	} else {
		_gatlas[a].relations[b] = TR;
	}
	// Insert the reversed transformation
	TransformData invTR;
	invTR.rot = TR.rot.inverse();
	invTR.t = invTR.rot * (-TR.t);

	if (_gatlas.count(b) == 0) {
		GAtlasElement el;
		el.id = b;
		el.relations.insert(std::pair<int, TransformData>(a, invTR)); 
		_gatlas.insert(std::pair<int, GAtlasElement>(b, el));
	} else {
		_gatlas[b].relations[a] = invTR;
	}

	_atlasdata_mx.unlock();

#ifdef GATLAS_DEBUG
	std::cout << "Transform [" << a << " --> " << b << "]:"<< std::endl; 
	std::cout << "      t = " << TR.t.transpose() << std::endl; 
	std::cout << "      q = " << TR.rot.w() << " " <<
		TR.rot.vec().transpose() << std::endl;

	std::cout << "Transform [" << b << " --> " << a << "]:"<< std::endl; 
	std::cout << "      t = " << invTR.t.transpose() << std::endl; 
	std::cout << "      q = " << invTR.rot.w() << " " <<
		invTR.rot.vec().transpose() << std::endl;
#endif

}

std::vector<int> GAtlas::get_fiducialsIds() {
    std::vector<int> out;

	_atlasdata_mx.lock();
    for (auto el : _gatlas) {
        out.push_back(el.first);
    }
	_atlasdata_mx.unlock();

    return out;
}


bool GAtlas::getTransform(int orig, int dest, TransformData& tr) {
	bool success = false;

	std::unique_lock<std::mutex> lk(_atlasdata_mx);

	// Check whether the nodes are in the map
	if (_gatlas.count(orig) == 0 || _gatlas.count(dest) == 0) {
		return false;
	}

	_visited.clear(); 
	if (find_path(orig, dest, tr)) {
		success = true;;
	}
	// Just to be sure...
	_visited.clear();

	lk.unlock();

	return success;
}


std::unordered_map<int, TargetData> GAtlas::getMap() {
	std::lock_guard<std::mutex> {_targetdata_mx};
	return _targetdata;	
}


int GAtlas::get_items(std::vector<TargetData>& vout) {
	std::lock_guard<std::mutex> {_targetdata_mx};
	// Download the full map and update the elements

	int counter = 0;
	for (auto el : _targetdata) {
		vout.push_back(el.second);
		counter++;
	}
	return counter;
}

// PRIVATE
bool GAtlas::find_path(int s, int e, TransformData& TR) {
	bool out = false;

	// Add the current node to the set of visited nodes
	_visited.insert(s);

	if (s == e) {
#ifdef GATLAS_DEBUG
			std::cout << "Path [" << s << " --> "<< e << "]:" <<
				std::endl; 
#endif
			return true;
	}

	// Nodes connected to the current node 's' 
	std::unordered_map<int, TransformData> rels = _gatlas[s].relations;


	// For each connected node
	for (auto el : rels) {
		int nextId = el.first;

		if (_visited.count(nextId) > 0) 
			continue;

		TransformData locTR = el.second;

		if (nextId == e) {
			TR.rot = locTR.rot;
			TR.t = locTR.t;
#ifdef GATLAS_DEBUG
			std::cout << "Path [" << s << " --> "<< e << "]:" <<
				std::endl; 
			std::cout << "      t:" << locTR.t.transpose() <<
				std::endl; 
			std::cout << "      q:" << locTR.rot.w() << " " <<
				locTR.rot.vec().transpose() << std::endl;
#endif
			return true;
		}

		TransformData downstreamTR;
		out = find_path(nextId, e, downstreamTR);
		if (out) {
			TR.t = locTR.rot * downstreamTR.t + locTR.t;
			TR.rot = locTR.rot * downstreamTR.rot;
#ifdef GATLAS_DEBUG
			std::cout << "Path [" << s << " --> "<< e << "]:" <<
				std::endl; 
			std::cout << "      t:" << TR.t.transpose() <<
				std::endl; 
			std::cout << "      q:" << TR.rot.w() << " " <<
				TR.rot.vec().transpose() << std::endl;
#endif
			return true;
		}
	}
	return false;
}
