# MiR - mir functions v1.0.0
"""
LEGAL DISCLAIMER

This example file is provided as is without any guarantees or warranty.
In association with the product, Mobile Industrial Robots ApS makes no warranties
of any kind, either express or implied, including but not limited to warranties
of merchantability, fitness for a particular purpose, of title, or of noninfringement
of third party rights. Use of the product by a user is at the user's risk.

NO REPRESENTATIONS OR WARRANTIES, EITHER EXPRESS OR IMPLIED, OF MERCHANTABILITY,
FITNESS FOR A SPECIFIC PURPOSE, THE PRODUCTS TO WHICH THE INFORMATION MENTIONS MAY BE
USED WITHOUT INFRINGING THE INTELLECTUAL PROPERTY RIGHTS OF OTHERS, OR OF ANY OTHER
NATURE ARE MADE WITH RESPECT TO INFORMATION OR THE PRODUCT TO WHICH INFORMATION MENTIONS.
IN NO CASE SHALL THE INFORMATION BE CONSIDERED A PART OF OUR TERMS AND CONDITIONS OF SALE.
"""

#Use 'from .' for python versions later than 2.7.13
import rest_urllib as rest
#import rest_requests as rest


class MiR(object):
    STATUS_OK = 200

    # Connect to MiR REST
    def __init__(self, host):
        self._client = rest.Client(host, port=8080, version="v1.0.0")

    # Pause the robot
    def pause_robot(self):
        data = {'state': 4}
        url = 'state'
        status, data = self._client.put(url, data=data, allow_insert=False)
        if status == self.STATUS_PUT_OK:
            return True
        else:
            return False

    # Resume robot operation
    def continue_robot(self):
        data = {'state': 3}
        url = 'state'
        status, data = self._client.put(url, data=data, allow_insert=False)
        if status == self.STATUS_PUT_OK:
            return True
        else:
            return False

    # Display all registers
    def all_registers(self):
        url = 'registers'
        status, data = self._client.get(url)
        if status == self.STATUS_OK:
            return True, data
        else:
            return False, data

    # Display value from 'register'
    def read_register(self, register):
        properties = 'value'
        url = 'registers/' + str(register)
        status, data = self._client.get(url)
        if status == self.STATUS_OK:
            return True, data[properties]
        else:
            return False, 0

    # Write 'value' to 'register'
    def write_register(self, register_id, register_value):
        if register_id in range(1, 201):
            data = {'value': register_value}
            url = 'registers/' + str(register_id)
            status, data = self._client.put(url, data=data)
            if status == self.STATUS_PUT_OK:
                return True
            else:
                return False
        else:
            raise ValueError("The registered id number: %d does not exist. Valid inputs are [1-200]." % register_id)

    # Display the current status of the robot
    def status_overview(self):
        url = 'status'
        status, data = self._client.get(url)
        if status == self.STATUS_OK:
            return True, data
        else:
            return False, data

    # Display robot name
    def robot_name(self):
        properties = 'robot_name'
        url = 'status'
        status, data = self._client.get(url)
        if status == self.STATUS_OK:
            return True, data[properties]
        else:
            return False, ''

    # Display percentage of remaining battery
    def battery_percentage(self):
        properties = 'battery_percentage'
        url = 'status'
        status, data = self._client.get(url)
        if status == self.STATUS_OK:
            return True, data[properties]
        else:
            return False, -1

    # Move robot to position
    def move_to_position(self, position_id):
        data = {'taxi': position_id}
        url = 'mission_queue'
        status, data = self._client.post(url, data=data)
        if status ==  self.STATUS_POST_OK:
            return True
        else:
            return False

    # Append mission to mission queue
    def load_mission(self, mission_id):
        data = {'mission': mission_id}
        url = 'mission_queue'
        status, data = self._client.post(url, data=data)
        if status == self.STATUS_POST_OK:
            return True, data['id']
        else:
            return False, -1

    # Read the state of a mission with the given id
    def read_mission_id_state(self, id):
        properties = 'state'
        url = 'mission_queue/' + str(id)
        status, data = self._client.get(url)
        if status == self.STATUS_OK:
            return True, data[properties]
        else:
            return False, -1

    # Delete mission from the queue
    def delete_from_mission_queue(self, guid):
        url = 'mission_queue/' + str(guid)
        status, data = self._client.delete(url)
        if status == self.STATUS_OK:
            return True
        else:
            return False

    # Display robot state ID
    def robot_state_id(self):
        properties = 'state'
        url = 'state'
        status, data = self._client.get(url)
        if status == self.STATUS_OK:
            return True, data[properties]
        else:
            return False, -1

    # Display robot state text
    def robot_state_text(self):
        properties = 'state_string'
        url = 'state'
        status, data = self._client.get(url)
        if status == self.STATUS_OK:
            return True, data[properties]
        else:
            return False, ''

    # Display list of missions in the robot
    def get_missions(self):
        url = 'missions?urls=false'
        status, data = self._client.get(url)
        if status == self.STATUS_OK:
            return True, data
        else:
            return False, data

    # Display mission GUID
    def get_mission_guid(self, name):
        valid, missions = self.get_missions()
        if valid:
            for x in missions:
                if x['name'] == name:
                    return True, x['guid']
        else:
            return False, ''

    # Display list of positions in the robot
    def get_positions(self):
        url = 'positions?urls=false'
        status, data = self._client.get(url)
        if status == self.STATUS_OK:
            return True, data
        else:
            return False, data

    # Display position GUID
    def get_position_guid(self, name):
        valid, positions = self.get_positions()
        if valid:
            for x in positions:
                if x['name'] == name:
                    return True, x['guid']
        else:
            return False, ''

#TODO: how to get mission and position ID
