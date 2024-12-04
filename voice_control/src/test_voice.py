#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import speech_recognition as sr
import pyttsx3

class VoiceControlledTurtleBot:
    def __init__(self):
        rospy.init_node('voice_controller')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.recognizer = sr.Recognizer()
        self.engine = pyttsx3.init()
        self.rate = rospy.Rate(10)
        self.move_commands = {
            'forward': (0.2, 0.0),
            'backward': (-0.2, 0.0),
            'left': (0.0, 0.2),
            'right': (0.0, -0.2),
            'stop': (0.0, 0.0)
        }

    def speak(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def move_robot(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

    def process_command(self, command):
        command = command.lower()
        for key in self.move_commands:
            if key in command:
                linear, angular = self.move_commands[key]
                self.move_robot(linear, angular)
                self.speak(f"Moving {key}")
                return True
        return False

    def listen(self):
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)
            print("Listening...")
            try:
                audio = self.recognizer.listen(source)
                command = self.recognizer.recognize_google(audio)
                print(f"Recognized: {command}")
                if not self.process_command(command):
                    self.speak("Command not recognized")
            except sr.UnknownValueError:
                print("Could not understand audio")
            except sr.RequestError as e:
                print(f"Could not request results; {e}")

    def run(self):
        self.speak("Voice control activated")
        while not rospy.is_shutdown():
            self.listen()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = VoiceControlledTurtleBot()
        controller.run()
    except rospy.ROSInterruptException:
        pass
