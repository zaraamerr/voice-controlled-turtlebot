#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import speech_recognition as sr
import pyttsx3
import threading
import queue
import logging

class VoiceControlledTurtleBot:
    def __init__(self):
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s: %(message)s',
            handlers=[
                logging.FileHandler('turtlebot_voice_control.log'),
                logging.StreamHandler()  # Outputs to terminal
            ]
        )
        self.logger = logging.getLogger(__name__)

        rospy.init_node('voice_controller')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Speech recognition and synthesis setup
        self.recognizer = sr.Recognizer()
        self.engine = pyttsx3.init()
        
        # Configure speech engine for clarity and speed
        self.engine.setProperty('rate', 150)  # Faster speech rate
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[1].id)  # Select a different voice (usually clearer)
        
        # More descriptive movement commands
        self.move_commands = {
            'forward': (0.3, 0.0, "Moving forward"),
            'backward': (-0.3, 0.0, "Moving backward"),
            'left turn': (0.0, 0.5, "Turning left"),
            'right turn': (0.0, -0.5, "Turning right"),
            'stop': (0.0, 0.0, "Stopping")
        }

    def log_and_speak(self, message, log_level=logging.INFO):
        """Log message and speak it out loud"""
        self.logger.log(log_level, message)
        threading.Thread(target=self._speak_thread, args=(message,)).start()

    def _speak_thread(self, text):
        """Thread-safe speech output"""
        try:
            self.engine.say(text)
            self.engine.runAndWait()
        except Exception as e:
            self.logger.error(f"Speech error: {e}")

    def move_robot(self, linear_vel, angular_vel, action_description):
        """Execute robot movement and log the action"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        # Log and announce the action
        self.log_and_speak(action_description)
        
        # Publish movement command
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(0.1)  # Small delay for command execution

    def process_command(self, command):
        """Process voice commands and execute corresponding actions"""
        command = command.lower()
        
        # Log raw command
        self.logger.info(f"Received command: {command}")
        
        for key, (linear, angular, description) in self.move_commands.items():
            if key in command:
                # Execute movement with specific description
                self.move_robot(linear, angular, description)
                return True
        
        # Handle unrecognized commands
        self.log_and_speak("Sorry, I did not understand that command", 
                            log_level=logging.WARNING)
        return False

    def listen_continuously(self):
        """Continuous voice listening method"""
        with sr.Microphone() as source:
            # Initial noise adjustment
            self.log_and_speak("Adjusting microphone, please wait quietly")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            
            # Announce ready state
            self.log_and_speak("Ready for voice commands")

            while not rospy.is_shutdown():
                try:
                    self.logger.info("Listening for command...")
                    audio = self.recognizer.listen(source, timeout=3, phrase_time_limit=2)
                    
                    # Recognize speech
                    command = self.recognizer.recognize_google(audio)
                    
                    # Process recognized command
                    self.process_command(command)
                
                except sr.UnknownValueError:
                    self.logger.warning("Could not understand audio")
                except sr.RequestError as e:
                    self.log_and_speak(f"Speech recognition service error: {e}", 
                                        log_level=logging.ERROR)
                except sr.WaitTimeoutError:
                    pass  # Normal when no speech is detected

    def run(self):
        """Main run method to start voice control"""
        self.log_and_speak("TurtleBot voice control system activated")
        
        # Use threading for continuous listening
        listener_thread = threading.Thread(target=self.listen_continuously)
        listener_thread.daemon = True
        listener_thread.start()
        
        rospy.spin()

def main():
    try:
        controller = VoiceControlledTurtleBot()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

if __name__ == '__main__':
    main()

