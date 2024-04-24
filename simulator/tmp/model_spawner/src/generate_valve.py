#!/usr/bin/env python3
import glob, os
from math import pi

class Valve:
    def __init__(
        self,
        model_path,
        x_offset,
        y_offset,
        z_offset,
        roll_offset,
        pitch_offset,
        yaw_offset,
        pipe_radius,
        pipe_length,
        valve_pipe_radius,
        valve_pipe_length,
        valve_radius
    ):
        self.model_path = model_path
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.roll_offset = roll_offset
        self.pitch_offset = pitch_offset
        self.yaw_offset = yaw_offset
        self.connection_length = 0.1
        self.connection_radius = pipe_radius * 1.2
        self.pipe_radius = pipe_radius
        self.pipe_length = pipe_length
        self.valve_pipe_radius = valve_pipe_radius
        self.valve_pipe_length = valve_pipe_length
        self.valve_wheel_radius = valve_pipe_radius * 2
        self.valve_wheel_length = 0.05
        self.valve_radius = valve_radius

    def write_beginning(self, f):
        f.write('<?xml version="1.0" ?>\n')
        f.write('<sdf version="1.4">\n')
        f.write('   <model name="valve">\n')
        f.write(f"      <pose>{self.x_offset} {self.y_offset} {self.z_offset} {self.roll_offset} {self.pitch_offset} {self.yaw_offset}</pose>\n")
        return f

    def write_end(self, f):
        f.write("       <static>true</static>\n")
        f.write("   </model>\n")
        f.write("</sdf>\n")
        return f

    def write_pipe_connection(self, f, z_magnitude):
        name = 'connection' + str(z_magnitude)

        f.write(f'      <link name="{name}">\n')
        f.write(f"          <pose>0 0 {z_magnitude * self.pipe_length/2 - z_magnitude * self.connection_length/2} 0 0 0</pose>\n")
        f.write(f'          <visual name="{name}">\n')
        f.write("               <geometry>\n")
        f.write("                   <cylinder>\n")
        f.write(f"                      <radius>{self.connection_radius}</radius>\n")
        f.write(f"                      <length>{self.connection_length}</length>\n")
        f.write("                   </cylinder>\n")
        f.write("               </geometry>\n")
        f.write('               <material>\n')
        f.write('                   <name>Red</name>\n')
        f.write('                   <ambient>0.05 0.05 0.0 1.0</ambient>\n')
        f.write('                   <diffuse>0.5 0.5 0.4 1.0</diffuse>\n')
        f.write('                   <specular>0.7 0.7 0.04 1.0</specular>\n')
        f.write('                   <emissive>0.4 0.4 0.2 1.0</emissive>\n')
        f.write("               </material>\n")
        f.write("           </visual>\n")
        f.write(f'          <collision name="{name}">\n')
        f.write("               <geometry>\n")
        f.write("                   <cylinder>\n")
        f.write(f"                      <radius>{self.connection_radius}</radius>\n")
        f.write(f"                      <length>{self.connection_length}</length>\n")
        f.write("                   </cylinder>\n")
        f.write("               </geometry>\n")
        f.write("           </collision>\n")
        f.write("           <inertial>\n")
        f.write('               <mass><value>10</value></mass>\n')
        f.write('               <inertia>\n')
        f.write('                   <ixx>0.4</ixx><ixy>0.0</ixy><ixz>0.0</ixz><iyy>0.4</iyy><iyz>0.0</iyz><izz>0.2</izz>\n')
        f.write('               </inertia>\n')
        f.write("           </inertial>\n")
        f.write("       </link>\n")
        return f
    
    def write_pipe(self, f):
        f.write('       <link name="pipe">\n')
        f.write(f"          <pose>0 0 0 0 0 0</pose>\n")
        f.write('           <visual name="pipe">\n')
        f.write("               <geometry>\n")
        f.write("                   <cylinder>\n")
        f.write(f"                      <radius>{self.pipe_radius}</radius>\n")
        f.write(f"                      <length>{self.pipe_length - 2*self.connection_length}</length>\n")
        f.write("                   </cylinder>\n")
        f.write("               </geometry>\n")
        f.write('               <material>\n')
        f.write('                   <name>Red</name>\n')
        f.write('                   <ambient>0.05 0.05 0.0 1.0</ambient>\n')
        f.write('                   <diffuse>0.5 0.5 0.4 1.0</diffuse>\n')
        f.write('                   <specular>0.7 0.7 0.04 1.0</specular>\n')
        f.write('                   <emissive>0.4 0.4 0.2 1.0</emissive>\n')
        f.write("               </material>\n")
        f.write("           </visual>\n")
        f.write('           <collision name="pipe">\n')
        f.write("               <geometry>\n")
        f.write("                   <cylinder>\n")
        f.write(f"                      <radius>{self.pipe_radius}</radius>\n")
        f.write(f"                      <length>{self.pipe_length - 2*self.connection_length}</length>\n")
        f.write("                   </cylinder>\n")
        f.write("               </geometry>\n")
        f.write("           </collision>\n")
        f.write("           <inertial>\n")
        f.write('               <mass><value>10</value></mass>\n')
        f.write('               <inertia>\n')
        f.write('                   <ixx>0.4</ixx><ixy>0.0</ixy><ixz>0.0</ixz><iyy>0.4</iyy><iyz>0.0</iyz><izz>0.2</izz>\n')
        f.write('               </inertia>\n')
        f.write("           </inertial>\n")
        f.write("       </link>\n")
        return f
    
    def write_valve(self, f):
        f.write('       <link name="valve_pipe">\n')
        f.write(f"          <pose>{self.valve_pipe_length/2} 0 0 0 {pi/2} 0</pose>\n")
        f.write('           <visual name="valve_pipe">\n')
        f.write("               <geometry>\n")
        f.write("                   <cylinder>\n")
        f.write(f"                      <radius>{self.valve_pipe_radius}</radius>\n")
        f.write(f"                      <length>{self.valve_pipe_length}</length>\n")
        f.write("                   </cylinder>\n")
        f.write("               </geometry>\n")
        f.write('               <material>\n')
        f.write('                   <name>Red</name>\n')
        f.write('                   <ambient>0.05 0.05 0.0 1.0</ambient>\n')
        f.write('                   <diffuse>0.5 0.5 0.4 1.0</diffuse>\n')
        f.write('                   <specular>0.7 0.7 0.04 1.0</specular>\n')
        f.write('                   <emissive>0.4 0.4 0.2 1.0</emissive>\n')
        f.write("               </material>\n")
        f.write("           </visual>\n")
        f.write('           <collision name="valve_pipe">\n')
        f.write("               <geometry>\n")
        f.write("                   <cylinder>\n")
        f.write(f"                      <radius>{self.valve_pipe_radius}</radius>\n")
        f.write(f"                      <length>{self.valve_pipe_length}</length>\n")
        f.write("                   </cylinder>\n")
        f.write("               </geometry>\n")
        f.write("           </collision>\n")
        f.write("           <inertial>\n")
        f.write('               <mass><value>10</value></mass>\n')
        f.write('               <inertia>\n')
        f.write('                   <ixx>0.4</ixx><ixy>0.0</ixy><ixz>0.0</ixz><iyy>0.4</iyy><iyz>0.0</iyz><izz>0.2</izz>\n')
        f.write('               </inertia>\n')
        f.write("           </inertial>\n")
        f.write("       </link>\n")

        f.write('       <link name="valve_wheel">\n')
        f.write(f"          <pose relative_to='valve'>{self.valve_pipe_length} 0 0 0 {pi/2} 0</pose>\n")
        f.write('           <visual name="valve_pipe">\n')
        f.write("               <geometry>\n")
        f.write("                   <cylinder>\n")
        f.write(f"                      <radius>{self.valve_wheel_radius}</radius>\n")
        f.write(f"                      <length>{self.valve_wheel_length}</length>\n")
        f.write("                   </cylinder>\n")
        f.write("               </geometry>\n")
        f.write('               <material>\n')
        f.write('                   <name>Red</name>\n')
        f.write('                   <ambient>0.05 0.0 0.0 1.0</ambient>\n')
        f.write('                   <diffuse>0.5 0.4 0.4 1.0</diffuse>\n')
        f.write('                   <specular>0.7 0.04 0.04 1.0</specular>\n')
        f.write('                   <emissive>0.5 0.2 0.2 1.0</emissive>\n')
        f.write("               </material>\n")
        f.write("           </visual>\n")
        f.write('           <collision name="valve_wheel">\n')
        f.write("               <geometry>\n")
        f.write("                   <cylinder>\n")
        f.write(f"                      <radius>{self.valve_wheel_radius}</radius>\n")
        f.write(f"                      <length>{self.valve_wheel_length}</length>\n")
        f.write("                   </cylinder>\n")
        f.write("               </geometry>\n")
        f.write("           </collision>\n")
        f.write("           <inertial>\n")
        f.write('               <mass><value>10</value></mass>\n')
        f.write('               <inertia>\n')
        f.write('                   <ixx>0.4</ixx><ixy>0.0</ixy><ixz>0.0</ixz><iyy>0.4</iyy><iyz>0.0</iyz><izz>0.2</izz>\n')
        f.write('               </inertia>\n')
        f.write("           </inertial>\n")
        f.write("       </link>\n")
        return f

    def generate_sdf(self):
        try:
            f = open(self.model_path + "/model.sdf", "w")
        except Exception as e:
            print('Could not open file', e)

        f = self.write_beginning(f)
        f = self.write_pipe_connection(f, -1)
        f = self.write_pipe(f)
        f = self.write_pipe_connection(f, 1)
        f = self.write_valve(f)
        f = self.write_end(f)


if __name__ == "__main__":
    model_path = "valve"
    x_offset = 0
    y_offset = 0
    z_offset = 0
    roll_offset = 0
    pitch_offset = 0
    yaw_offset = 0
    pipe_radius = 0.2
    pipe_length = 1
    valve_pipe_radius = 0.2
    valve_pipe_length = 0.5
    valve_radius = 0.2

    valve = Valve(
        model_path = model_path,
        x_offset = x_offset,
        y_offset = y_offset,
        z_offset = z_offset,
        roll_offset = roll_offset,
        pitch_offset = pitch_offset,
        yaw_offset = yaw_offset,
        pipe_radius = pipe_radius,
        pipe_length = pipe_length,
        valve_pipe_radius = valve_pipe_radius,
        valve_pipe_length = valve_pipe_length,
        valve_radius = valve_radius
    )

    valve.generate_sdf()
