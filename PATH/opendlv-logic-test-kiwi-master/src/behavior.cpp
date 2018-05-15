/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "path.hpp"
#include "behavior.hpp"
#include <iostream>
#include <tgmath.h>

Behavior::Behavior() noexcept:
  m_nodes{},
  m_node{},
  counter{2},
  m_frontUltrasonicReading{},
  m_rearUltrasonicReading{},
  m_leftIrReading{},
  m_rightIrReading{},
  m_groundSteeringAngleRequest{},
  m_pedalPositionRequest{},
  m_pos{},
  m_frontUltrasonicReadingMutex{},
  m_rearUltrasonicReadingMutex{},
  m_leftIrReadingMutex{},
  m_rightIrReadingMutex{},
  m_groundSteeringAngleRequestMutex{},
  m_pedalPositionRequestMutex{},
  m_posMutex{},
  state{'A'}
{
}

opendlv::proxy::GroundSteeringRequest Behavior::getGroundSteeringAngle() noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleRequestMutex);
  return m_groundSteeringAngleRequest;
}

opendlv::proxy::PedalPositionRequest Behavior::getPedalPositionRequest() noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionRequestMutex);
  return m_pedalPositionRequest;
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::VoltageReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::VoltageReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}

void Behavior::setPos(opendlv::sim::Frame const &pos) noexcept
{
  std::lock_guard<std::mutex> lock(m_posMutex);
  m_pos = pos;
}

void Behavior::setNodes(double src_x, double src_y, double goal_x, double goal_y) noexcept
{
  Path path;
  m_nodes = path.getNode(src_x,src_y,goal_x,goal_y);
  m_node = m_nodes[1];
}



void Behavior::step(float FORWARD_SPEED, float TURNSPEED_ANGLE, float TURN_ANGLE, float REVERSE_SPEED, float REVERSETURNSPEED_ANGLE, float REVERSETURN_ANGLE) noexcept
{
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::VoltageReading leftIrReading;
  opendlv::proxy::VoltageReading rightIrReading;
  opendlv::sim::Frame pos;
  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);
    std::lock_guard<std::mutex> lock5(m_posMutex);

    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
    pos = m_pos;
  }

  float frontDistance = frontUltrasonicReading.distance();
  float rearDistance = rearUltrasonicReading.distance();
  double leftDistance = convertIrVoltageToDistance(leftIrReading.voltage());
  double rightDistance = convertIrVoltageToDistance(rightIrReading.voltage());
  double x_pos = pos.x();
  double y_pos = pos.y();
  double yaw_pos = pos.yaw();

  float pedalPosition = 0.0f;
  float groundSteeringAngle = 0.0f;
  double random = (double) rand()/ (RAND_MAX) ;

// State A = FORWARD
// State B = TURN LEFT
// STATE C = TURN RIGHT
// STATE D = REVERSE
// STATE E = REVERSE LEFT (CLOCKWISE)
// STATE F = REVERSE RIGHT (COUNTERCLOCKWISE)	

  switch(state){
	case 'A':

      		if( frontDistance < 0.3 || (frontDistance < 0.5 && leftDistance < 0.3 && rightDistance < 0.3)){
			state = 'D';
		}else if(frontDistance < 0.8){
			if(leftDistance > 0.4 && 0.4 < rightDistance){
				if(0.5 > random){
					state = 'B';
				}
				 else{
					state = 'C';
				}  
			}else if(leftDistance > rightDistance){
				state = 'B';
			}
			else{
				state = 'C';
			}

		}else if(leftDistance < 0.25){
			state = 'C';
		}else if(rightDistance < 0.25){
			state = 'B';
		}else{
			pedalPosition = FORWARD_SPEED;
  			groundSteeringAngle = 0.0f;
		}

		break;

	case 'B':
		if(frontDistance < 0.4){
			state = 'F';
		}
		else if(frontDistance > 0.8 && rightDistance > 0.3){
			state = 'A';
		}else if(rightDistance > (leftDistance + 0.01) ){
			state = 'C';
		}else{
			pedalPosition = TURNSPEED_ANGLE;
  			groundSteeringAngle = TURN_ANGLE;
		}
		break;

	case 'C':
		if(frontDistance < 0.4){
			state = 'E';
		}
		else if(frontDistance > 0.8 && leftDistance > 0.3){
			state = 'A';
		}else if(leftDistance > (rightDistance+0.01)){
			state = 'B';
		}else{
			pedalPosition = TURNSPEED_ANGLE;
  			groundSteeringAngle = -TURN_ANGLE;
		}

		break;

	case 'D':
		if(rearDistance < 0.3){
			if(frontDistance > rearDistance)
				state = 'A';
		}else if(leftDistance > 0.3 && frontDistance > 0.2){
			state = 'E';
		}else if(rightDistance > 0.3 &&  frontDistance > 0.2){
			state = 'F';
		}else{
			pedalPosition = -REVERSE_SPEED;
  			groundSteeringAngle = 0.0f;
		}
		break;

	case 'E':
		if(rearDistance < 0.4){
			if(frontDistance > rearDistance)
				state = 'A';
		}else if(frontDistance > 0.7){
			state = 'C';
		}else if(rightDistance > leftDistance){
			state = 'F';
		}else{
			pedalPosition = -REVERSETURNSPEED_ANGLE;
  			groundSteeringAngle = -REVERSETURN_ANGLE;
		}
		break;

	case 'F':
		if(rearDistance < 0.4){
			if(frontDistance > rearDistance)
				state = 'A';
		}else if(frontDistance > 0.7){
			state = 'B';
		}else if(leftDistance > rightDistance){
			state = 'E';
		}else{
			pedalPosition = -REVERSETURNSPEED_ANGLE;
  			groundSteeringAngle = REVERSETURN_ANGLE;
		}
		break;

		default :
			pedalPosition = 0.0f;
			groundSteeringAngle = 0.0f;
  }

  {
    std::lock_guard<std::mutex> lock1(m_groundSteeringAngleRequestMutex);
    std::lock_guard<std::mutex> lock2(m_pedalPositionRequestMutex);

    opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest;
    groundSteeringAngleRequest.groundSteering(groundSteeringAngle);
    m_groundSteeringAngleRequest = groundSteeringAngleRequest;

    opendlv::proxy::PedalPositionRequest pedalPositionRequest;
    pedalPositionRequest.position(pedalPosition);
    m_pedalPositionRequest = pedalPositionRequest;
  }

  //std::cout << "F = "<< frontDistance << " B = " << rearDistance << " L = " << leftDistance << " R = " << rightDistance << " State = " << state  <<std::endl;
  std::cout << "x = " << x_pos << " y = " << y_pos << " yaw = " << yaw_pos << " Counter = " << counter << std::endl;
}

// TODO: This is a rough estimate, improve by looking into the sensor specifications.
double Behavior::convertIrVoltageToDistance(float voltage) const noexcept
{
  double voltageDividerR1 = 1000.0;
  double voltageDividerR2 = 1000.0;

  double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
  double distance = 0.001645*pow(sensorVoltage, 8) - 0.04791*pow(sensorVoltage,7) + 0.4231*pow(sensorVoltage,6) - 1.826*pow(sensorVoltage,5) + 4.495*pow(sensorVoltage,4) - 6.694*pow(sensorVoltage,3) + 6.115*pow(sensorVoltage,2) - 3.354*pow(sensorVoltage,1) + 1.016;
return distance;
  return distance;
}

void Behavior::updatePath() noexcept{
   if( sqrt(pow(m_pos.x()-m_node.x,2)+pow(m_pos.y()-m_node.y,2))  < 0.3 ){
      m_node = m_nodes[counter];
      counter++;
   }
}
