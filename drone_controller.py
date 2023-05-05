import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions



class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.get_logger().info("Node Iniciado")

        connection_string = "127.0.0.1:14550"

        # Conectando o drone ao endereço acima
        print('Connecting to vehicle on: %s' % connection_string)
        self.drone = connect(connection_string, wait_ready=True)

        self.drone.mode = VehicleMode("GUIDED")
        self.drone.armed = True
        print("Armando o Drone")

                                                                             # Inscrito ao topico da cor
        self.subscription = self.create_subscription(String, '/detected_color', self.color_callback, 10)

        self.previous_color = None
        self.started = False #indica inicializacao do drone
        self.done = False #indica percurso finalizado

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
       
        msg = self.drone.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            1, 1,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


        # Ciclo de 2hz para o drone
        for x in range(0,duration):
            self.drone.send_mavlink(msg)
            time.sleep(0.5)

    def color_callback(self, msg):
        # Para Controlar o drone de acordo com a cor

        #print(self.drone.mode)

        if not self.started and not self.done:
            if msg.data == "green":
                self.started = True
                self.previous_color = msg.data
                self.drone.simple_takeoff(1)
                print("Decolando!")
                time.sleep(15)
                print("Acordou")
                
            return
        
        elif msg.data == "green" and self.previous_color == "green":

            self.previous_color = msg.data
            print("[VERDE]: Seguindo percurso...")
            #self.goto_position_target_local_ned()
            #self.set_roi(self.drone.location.global_relative_frame)
            self.send_ned_velocity(-1, 0, 0, 1)
            time.sleep(5)

        elif msg.data == "blue": #and self.previous_color == "green":
            if self.previous_color == "green":
                print("[CHEGOU NO AZUL]: freando!")
                time.sleep(1)
            
            print("[AZUL]: Seguindo percurso...")
            self.previous_color = msg.data
            self.send_ned_velocity(0, -1, 0, 1)    
            time.sleep(5)

        elif msg.data == "pink": # and self.previous_color == "blue":
            if self.previous_color == "blue":
                print("[CHEGOU NO ROSA]: freando")
                time.sleep(1)
            print("[ROSA]: Seguindo percurso...")
            self.previous_color = msg.data
            self.send_ned_velocity(1,0,0,1)
            time.sleep(5)

        elif (msg.data == "red1" or msg.data == "red2"):
            if self.previous_color == "pink":
                print("[CHEGOU NO VERMELHO]: freando!")
                time.sleep(1)
            print("[VERMELHO]: Seguindo percurso...")
            self.previous_color = msg.data
            self.send_ned_velocity(0, 1, 0, 1) 
            time.sleep(5)

        elif (msg.data == "green" and self.previous_color in ["red1", "red2"] and self.started == True):
            if self.previous_color in ["red1", "red2"]:
                print("[Chegou no verde novamente]: Iniciando pouso")
            #self.drone.mode = VehicleMode("RTL")  # Decidi não utilizar do modo RTL, pois ele segue uma altura especifica
            self.send_ned_velocity(0,0,1,2) #Descende o drone
            time.sleep(5)       

            print("Desarmando")
            #reseta os valores de início
            self.drone.armed = False 
            self.previous_color = None
            self.started = False
            self.done = True # Acaba o percurso e proíbe de entrar em loop novamente

#-------------------------- FAILSAFE --------------------------------------------------------------------------------------------
        elif (msg.data not in ["red1", "red2", "green", "blue", "pink"]): #Failsafe
            print("Nenhuma cor detectada, voltando à base")
            while(self.drone.mode == VehicleMode("RTL") and self.drone.armed == True):
                print("Drone retornando à base...")
                time.wait(3)
            print("Failsafe ocorreu e o drone pousou")
#---------------------------------------------------------------------------------------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    drone_control = DroneControl()


    rclpy.spin(drone_control)
    
    drone_control.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()