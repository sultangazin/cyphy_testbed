#ifndef _RPC_DATA_HPP_
#define _RPC_DATA_HPP_ 

#include <time.h>
#include "rpc/msgpack.hpp"


struct RpcData {
	// Timestamp
	uint64_t t;
	// Id
	int id;

	//Position
	double x;
	double y;
	double z;

	MSGPACK_DEFINE_ARRAY(t, id, x, y, z);
};

struct RpcSynchData {
	timespec t;	
	MSGPACK_DEFINE(t);
};

#endif
