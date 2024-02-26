#!/usr/bin/python3
import paho.mqtt.client as mqtt
import pygame
from PIL import Image

global counter
counter = 0

# Dictionary to store the received image parts
parts = {}

# Initialize Pygame
pygame.init()

# Set the window size
window_size = (600, 400)
global frame_counter
frame_counter = 0

# Create a window
screen = pygame.display.set_mode(window_size)

def avg_brightness(image):

  # Convert the image to a 3D array of integers representing the red, green, and blue channel values for each pixel
  pixels = list(image.getdata())
  pixels = [pixels[i:i+image.width] for i in range(0, len(pixels), image.width)]

  # Calculate the average brightness of the image
  total_brightness = 0
  for row in pixels:
    for pixel in row:
      total_brightness += sum(pixel)
  average_brightness = total_brightness / (len(pixels) * len(pixels[0]))
  return average_brightness


def render():
    try:
      image_name='image.jpg'

      # Load the image from the file into a Pygame surface
      image = pygame.image.load(image_name)

      # Stretch the image to fit the window size
      image = pygame.transform.scale(image, window_size)

      # Clear the screen
      screen.fill((0, 0, 0))

      # Draw the image on the screen
      screen.blit(image, (0, 0))

      screen_surface = pygame.display.get_surface()
      global frame_counter
      frame_counter += 1
      if frame_counter % 10 == 0:
        print(avg_brightness(Image.open('image.jpg')))

      # Update the display
      pygame.display.flip()
    except Exception as e:
      print(e)
    #print('done')

def on_message(client, userdata, message):
  if "esp32cam/50/image/part/" in message.topic:
    # Store the received part in the dictionary
    part_num = int(message.topic.split("/")[-1])  # Extract the part number from the topic
    parts[part_num] = message.payload
    #print("Message size:", len(message.payload))
  elif message.topic == "esp32cam/50/image_complete":

    num_parts = int(message.payload)  # Get the number of parts from the message
    #print('image_complete with', num_parts, 'parts')

    #create a blank new image and write the first part
    with open("image.jpg", "wb") as f:
        f.write(parts[0])
        #print('wrote part 0')

    # Concatenate the rest of the parts and save the image
    with open("image.jpg", "ab") as f:  # Open the file in append mode
      for i in range(1, num_parts):
        f.write(parts[i])
        #print('wrote part', str(i))
    render()

render()

# Create a new MQTT client
client = mqtt.Client()

# Set the callback for when a message is received
client.on_message = on_message

# Connect to the MQTT broker
client.connect("localhost", 1883, 60)

# Subscribe to a topic
client.subscribe("esp32cam/50/#")

# Start the MQTT loop to receive messages
client.loop_forever()
