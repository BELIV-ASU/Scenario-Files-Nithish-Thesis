import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower,
                                                                      ActorSource, 
                                                                      ActorSink,
                                                                      CustomWait)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance

from agents.navigation.global_route_planner import GlobalRoutePlanner

import time

class CustomWaypointFollowerScenario2(BasicScenario):
    timeout = 120

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        
        self._map = CarlaDataProvider.get_map()
        '''
        self._other_actor_vehicle_speed = [5.0, 5.0, 5.0, 5.0, 5.0]
        self._other_actor_max_brake = [0.25, 0.5, 0.5, 0.5, 0.5]
        '''
        self._other_actor_vehicle_speed = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5, 5.5]
        self._other_actor_max_brake = [0.0, 0.05, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        # Timeout of scenario in seconds
        self.timeout = timeout

        self.client = carla.Client('localhost', 2000)
        self.world = self.client.get_world()

        super(CustomWaypointFollowerScenario2, self).__init__("CustomWaypointFollowerScenario2",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode=debug_mode,
                                                   criteria_enable=criteria_enable)
        
    def waypoint_visualizer(self, waypoints_for_this_vehicle, color, thickness=1, offset=0.1, life_time=10):
        for waypoint in waypoints_for_this_vehicle:
            for dx in range(-thickness, thickness + 1):
                for dy in range(-thickness, thickness + 1):
                    offset_location = carla.Location(waypoint.x + dx * offset, waypoint.y + dy * offset, 0.1)
                    self.world.debug.draw_string(offset_location, '*', draw_shadow=False, color=color, life_time=life_time, persistent_lines=True)


    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        #print("config.other_actors: ", config.other_actors)
        self.vehicle_start_location = [0 for i in range(int(len(config.other_actors)/2))]
        self.vehicle_end_location = [0 for i in range(int(len(config.other_actors)/2))]
        self.vehicle_start_waypoint = [0 for i in range(int(len(config.other_actors)/2))]
        self.vehicle_start_waypoint_transform = [0 for i in range(int(len(config.other_actors)/2))]
        self.vehicle_end_waypoint = [0 for i in range(int(len(config.other_actors)/2))]
        self.vehicle_end_waypoint_transform = [0 for i in range(int(len(config.other_actors)/2))]
        self.other_vehicles = [0 for i in range(int(len(config.other_actors)/2))]
        self.vehicles_waypoints_list = []
        j=0
        for k in range(0, len(config.other_actors), 2):
            #print("config.other_actors{}.transform".format(j), config.other_actors[j].transform.location)
            #print("config.other_actors{}.model".format(j), config.other_actors[k].model)
            self.vehicle_start_location[j] = config.other_actors[k].transform.location
            self.vehicle_end_location[j] = config.other_actors[k+1].transform.location
            self.vehicle_start_waypoint[j] = self._map.get_waypoint(self.vehicle_start_location[j])
            self.vehicle_start_waypoint_transform[j] = self.vehicle_start_waypoint[j].transform
            self.vehicle_start_waypoint_transform[j].location.z += 0.5
            self.vehicle_end_waypoint[j] = self._map.get_waypoint(self.vehicle_end_location[j])
            self.vehicle_end_waypoint_transform[j] = self.vehicle_end_waypoint[j].transform
            self.vehicle_end_waypoint_transform[j].location.z += 0.5
            self.other_vehicles[j] = CarlaDataProvider.request_new_actor(config.other_actors[k].model, \
                                                    self.vehicle_start_waypoint_transform[j])
            self.other_actors.append(self.other_vehicles[j])

            waypoints_for_this_vehicle = self.waypoints_using_global_planner(self.vehicle_start_waypoint_transform[j].location, \
                                            self.vehicle_end_waypoint_transform[j].location)
            self.vehicles_waypoints_list.append(waypoints_for_this_vehicle)

            color = carla.Color(r=255, g=0, b=0)

            self.waypoint_visualizer(waypoints_for_this_vehicle, color, thickness=1, offset=0.1, life_time=40)

            j=j+1



    def waypoints_using_global_planner(self, point_a, point_b):
        sampling_resolution = 1
        town_map = self.world.get_map()
        grp = GlobalRoutePlanner(town_map, sampling_resolution)
        route = grp.trace_route(point_a, point_b)

        waypoint_list = []
        for waypoint in route:
            #print(waypoint[0])
            waypoint_list.append(waypoint[0].transform.location)
        
        return waypoint_list

        

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # let the other actor drive until next intersection
        driving_to_next_intersection = py_trees.composites.Parallel("DrivingTowardsIntersection", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        wait_time = 0.1

        waypoint_follower_0 = WaypointFollower(self.other_actors[0], self._other_actor_vehicle_speed[0], \
                                                self.vehicles_waypoints_list[0])
        wait_0 = CustomWait(wait_time=wait_time, actor_name="actor_0")
        stop_vehicle_0 = StopVehicle(self.other_actors[0], self._other_actor_max_brake[0])
        end_condition_0 = StandStill(self.other_actors[0], name="StandStill", duration=1)
        driving_to_next_intersection.add_child(py_trees.composites.Sequence("Vehicle 0 Sequence", \
                                                children=[waypoint_follower_0, wait_0, stop_vehicle_0]))  
        
        waypoint_follower_1 = WaypointFollower(self.other_actors[1], self._other_actor_vehicle_speed[1], \
                                                self.vehicles_waypoints_list[1])
        wait_1 = CustomWait(wait_time=wait_time, actor_name="actor_1")
        stop_vehicle_1 = StopVehicle(self.other_actors[1], self._other_actor_max_brake[1])
        end_condition_1 = StandStill(self.other_actors[1], name="StandStill", duration=1)
        driving_to_next_intersection.add_child(py_trees.composites.Sequence("Vehicle 1 Sequence", \
                    children=[waypoint_follower_1, wait_1, stop_vehicle_1]))

        
        waypoint_follower_2_0 = WaypointFollower(self.other_actors[2], self._other_actor_vehicle_speed[2], \
                                                self.vehicles_waypoints_list[2][:112])
        waypoint_follower_2_1 = WaypointFollower(self.other_actors[2], 6.9, \
                                                self.vehicles_waypoints_list[2][112:125])
        waypoint_follower_2_2 = WaypointFollower(self.other_actors[2], 8.3, \
                                                self.vehicles_waypoints_list[2][125:])
        wait_2 = CustomWait(wait_time=wait_time, actor_name="actor_2")
        stop_vehicle_2 = StopVehicle(self.other_actors[2], self._other_actor_max_brake[2])
        end_condition_2 = StandStill(self.other_actors[2], name="StandStill", duration=1)
        driving_to_next_intersection.add_child(py_trees.composites.Sequence("Vehicle 2 Sequence", \
                    children=[waypoint_follower_2_0, waypoint_follower_2_1, waypoint_follower_2_2, wait_2, stop_vehicle_2]))
        

        vehicle3_trigger_condition = InTriggerDistanceToVehicle(self.other_actors[3], self.other_actors[0], \
                                                distance=68, name="Vehicle 3 Trigger distance")
        waypoint_follower_3 = WaypointFollower(self.other_actors[3], self._other_actor_vehicle_speed[3], \
                                                self.vehicles_waypoints_list[3])
        wait_3 = CustomWait(wait_time=wait_time, actor_name="actor_3")
        stop_vehicle_3 = StopVehicle(self.other_actors[3], self._other_actor_max_brake[3])
        end_condition_3 = StandStill(self.other_actors[3], name="StandStill", duration=1)
        driving_to_next_intersection.add_child(py_trees.composites.Sequence("Vehicle 3 Sequence", \
                                                children=[vehicle3_trigger_condition, waypoint_follower_3, wait_3, stop_vehicle_3]))


        vehicle4_trigger_condition = InTriggerDistanceToVehicle(self.other_actors[4], self.other_actors[0], \
                                                distance=65, name="Vehicle 4 Trigger distance")
        waypoint_follower_4 = WaypointFollower(self.other_actors[4], self._other_actor_vehicle_speed[4], \
                                                self.vehicles_waypoints_list[4])
        wait_4 = CustomWait(wait_time=wait_time, actor_name="actor_4")
        stop_vehicle_4 = StopVehicle(self.other_actors[4], self._other_actor_max_brake[4])
        end_condition_4 = StandStill(self.other_actors[4], name="StandStill", duration=1)
        driving_to_next_intersection.add_child(py_trees.composites.Sequence("Vehicle 4 Sequence", \
                                                children=[vehicle4_trigger_condition, waypoint_follower_4, wait_4, stop_vehicle_4]))


        vehicle5_trigger_condition = InTriggerDistanceToVehicle(self.other_actors[5], self.other_actors[0], \
                                                distance=62, name="Vehicle 5 Trigger distance")
        waypoint_follower_5 = WaypointFollower(self.other_actors[5], self._other_actor_vehicle_speed[5], \
                                                self.vehicles_waypoints_list[5])
        wait_5 = CustomWait(wait_time=wait_time, actor_name="actor_5")
        stop_vehicle_5 = StopVehicle(self.other_actors[5], self._other_actor_max_brake[5])
        end_condition_5 = StandStill(self.other_actors[5], name="StandStill", duration=1)
        driving_to_next_intersection.add_child(py_trees.composites.Sequence("Vehicle 5 Sequence", \
                                                children=[vehicle5_trigger_condition, waypoint_follower_5, wait_5, stop_vehicle_5]))


        waypoint_follower_6 = WaypointFollower(self.other_actors[6], self._other_actor_vehicle_speed[6], \
                                                self.vehicles_waypoints_list[6])
        wait_6 = CustomWait(wait_time=wait_time, actor_name="actor_6")
        stop_vehicle_6 = StopVehicle(self.other_actors[6], self._other_actor_max_brake[6])
        end_condition_6 = StandStill(self.other_actors[6], name="StandStill", duration=1)
        driving_to_next_intersection.add_child(py_trees.composites.Sequence("Vehicle 6 Sequence", \
                                                children=[waypoint_follower_6, wait_6, stop_vehicle_6]))


        waypoint_follower_7 = WaypointFollower(self.other_actors[7], self._other_actor_vehicle_speed[7], \
                                                self.vehicles_waypoints_list[7])
        wait_7 = CustomWait(wait_time=wait_time, actor_name="actor_7")
        stop_vehicle_7 = StopVehicle(self.other_actors[7], self._other_actor_max_brake[7])
        end_condition_7 = StandStill(self.other_actors[7], name="StandStill", duration=1)
        driving_to_next_intersection.add_child(py_trees.composites.Sequence("Vehicle 7 Sequence", \
                                                children=[waypoint_follower_7, wait_7, stop_vehicle_7]))
        
        

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(driving_to_next_intersection)

        
        while True:
            try:
                self.ego_vehicle = self.other_actor = [actor for actor in self.world.get_actors() if 'mercedes.coupe_2020' in actor.type_id][0]
                print("self.ego_vehicle_in_the_world: ", self.ego_vehicle)
                if self.ego_vehicle is not None:
                    ego_vehicle_velocity = self.ego_vehicle.get_velocity()
                    velocity_x = ego_vehicle_velocity.x
                    velocity_y = ego_vehicle_velocity.y
                    velocity_z = ego_vehicle_velocity.z
                    speed = (velocity_x**2 + velocity_y**2 + velocity_z**2) ** 0.5
                    print("velocity of the ego vehicle in the world: ", ego_vehicle_velocity)
                    print("speed of the ego vehicle in the world: ", speed)
                    if speed>5.2:
                        break
            except:
                continue
        

        
        
        return driving_to_next_intersection

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.other_actors[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self.remove_all_actors()