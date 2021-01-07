import roslibpy
import time

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

print("Starting ros bridge...")
start_bridge_srv = roslibpy.Service(client, '/start_bridge', 'carla_autoware_manager/StartBridge')
result = start_bridge_srv.call(roslibpy.ServiceRequest())

print("Waiting 30 seconds...")
time.sleep(30)

print("Starting agent...")
start_agent_srv = roslibpy.Service(client, '/start_agent', 'carla_autoware_manager/StartAgent')
result = start_agent_srv.call(roslibpy.ServiceRequest())

print("Waiting 30 seconds...")
time.sleep(30)

print("Stopping agent...")
stop_agent_srv = roslibpy.Service(client, '/stop_agent', 'carla_autoware_manager/StopAgent')
result = stop_agent_srv.call(roslibpy.ServiceRequest())

print("Waiting 30 seconds...")
time.sleep(30)

print("Stopping ros bridge...")
stop_bridge_srv = roslibpy.Service(client, '/stop_bridge', 'carla_autoware_manager/StopBridge')
result = stop_bridge_srv.call(roslibpy.ServiceRequest())
