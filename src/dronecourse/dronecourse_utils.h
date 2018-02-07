#include <uORB/uORB.h>


template <typename T>
bool orb_fetch_all(const struct orb_metadata *meta, int sub_handles[], T *msg, int sub_count){
	uint64_t timestamp = 0;
	bool updated = false;
	T msg_tmp;
	for(int i=0; i < sub_count; i++){
		bool updated_i;
		orb_check(sub_handles[i], &updated_i);
		if(updated_i)
		{
			updated = true;
			orb_copy(meta, sub_handles[i], &msg_tmp);
			if(msg_tmp.timestamp > timestamp)
			{
				*msg = msg_tmp;
			}
		}
	}
	
	return updated;
}