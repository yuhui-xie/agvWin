#ifndef SENSORREADING_H
#define SENSORREADING_H

#include "sensor.h"
namespace GMapping{

class SensorReading{
	public:
		SensorReading(const Sensor* s=0, int time=0);
		virtual ~SensorReading();
		inline int getTime() const {return m_time;}
		inline void setTime(double t) {m_time=t;}
		inline const Sensor* getSensor() const {return m_sensor;}
	protected:
		int m_time;
		const Sensor* m_sensor;

};

}; //end namespace
#endif


