#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import tkinter as tk
import random
import subprocess
import csv
import os
from datetime import datetime

INTERFACE = "enp2s0"

TOTAL_DELAY = 20    
PHASE_TIME = 20      


class Experiment:

    def __init__(self):

        rospy.init_node('master_delay_node')
        self.pub = rospy.Publisher('/delay_cmd', Int32MultiArray, queue_size=10)

        self.root = tk.Tk()
        self.root.title("Delay Perception Experiment")
        self.root.geometry("1000x650")

        self.stage = "name"
        self.trial = 0
        self.phase_active = False

        # Generate ALL asymmetric pairs once
        self.asym_pairs = self.generate_all_pairs()
        random.shuffle(self.asym_pairs)

        self.label = tk.Label(self.root, text="Enter Name and Press ENTER",
                              font=("Arial", 24))
        self.label.pack(expand=True)

        self.entry = tk.Entry(self.root, font=("Arial", 20))
        self.entry.pack()
        self.entry.focus()

        self.root.bind("<Return>", self.handle_enter)
        self.root.bind("<Key>", self.key_handler)

    # ----------------------------------------
    def generate_all_pairs(self):
        pairs = []
        for i in range(1, TOTAL_DELAY):
            if i != TOTAL_DELAY - i:
                pairs.append((i, TOTAL_DELAY - i))
        return pairs

    # ----------------------------------------
    def handle_enter(self, event):
        if self.stage == "name":
            self.username = self.entry.get().strip()
            if not self.username:
                return

            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

            base_path = "/home/autonomous-lab/Desktop/delay"
            os.makedirs(base_path, exist_ok=True)

            self.filename = os.path.join(base_path,
                                        f"{self.username}_{timestamp}.csv")

            self.entry.pack_forget()
            self.stage = "ready"
            self.label.config(text="Press S to Start Trial")

    # ----------------------------------------
    def key_handler(self, event):

        # Start trial
        if self.stage == "ready" and event.char.lower() == 's':
            self.start_trial()

        # Move to next phase manually
        elif self.stage == "phase" and event.char.lower() == 'n':
            self.next_phase()

        # Answer
        elif self.stage == "response" and event.char in ['1', '2', '3']:
            self.user_answer = int(event.char)
            self.save_trial()
            self.stage = "ready"
            self.label.config(text="Saved.\nPress S for next trial")

    # ----------------------------------------
    def apply_delay(self, m, s):

        subprocess.call(f"sudo tc qdisc del dev {INTERFACE} root 2>/dev/null", shell=True)

        subprocess.call(
            f"sudo tc qdisc add dev {INTERFACE} root netem delay {m}ms",
            shell=True)

        msg = Int32MultiArray()
        msg.data = [m, s]
        self.pub.publish(msg)

    # ----------------------------------------
    def start_trial(self):

        if len(self.asym_pairs) == 0:
            self.label.config(text="All asymmetric cases used!")
            return

        self.trial += 1

        sym = (TOTAL_DELAY // 2, TOTAL_DELAY // 2)

        # Pick unique asymmetric pair
        self.asym_m, self.asym_s = self.asym_pairs.pop()

        # Build phases
        self.phases = [
            {"type": "sym", "delay": sym},
            {"type": "sym", "delay": sym},
            {"type": "asym", "delay": (self.asym_m, self.asym_s)}
        ]

        random.shuffle(self.phases)

        for i, p in enumerate(self.phases):
            if p["type"] == "asym":
                self.correct_phase = i + 1

        self.current_phase = 0
        self.run_phase()

    # ----------------------------------------
    def run_phase(self):

        if self.current_phase >= 3:
            self.ask_response()
            return

        self.stage = "phase"
        self.phase_active = True

        phase = self.phases[self.current_phase]
        m, s = phase["delay"]

        self.apply_delay(m, s)

        self.label.config(
            text=f"Phase {self.current_phase + 1}\nExplore...\nPress N for next phase"
        )

        # Auto move after PHASE_TIME
        self.root.after(PHASE_TIME * 1000, self.auto_next_phase)

    # ----------------------------------------
    def auto_next_phase(self):
        if self.phase_active:
            self.next_phase()

    # ----------------------------------------
    def next_phase(self):
        self.phase_active = False
        self.current_phase += 1
        self.run_phase()

    # ----------------------------------------
    def ask_response(self):

        self.stage = "response"

        self.label.config(
            text="Which phase is DIFFERENT?\n\nPress 1 / 2 / 3"
        )

    # ----------------------------------------
    def save_trial(self):

        file_exists = os.path.isfile(self.filename)

        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)

            if not file_exists:
                writer.writerow([
                    "Timestamp", "User", "Trial",
                    "TotalDelay",
                    "Phase1", "Phase2", "Phase3",
                    "Asym_Master", "Asym_Slave",
                    "CorrectPhase", "UserAnswer"
                ])

            writer.writerow([
                datetime.now(),
                self.username,
                self.trial,
                TOTAL_DELAY,
                self.phases[0]["delay"],
                self.phases[1]["delay"],
                self.phases[2]["delay"],
                self.asym_m,
                self.asym_s,
                self.correct_phase,
                self.user_answer
            ])

    # ----------------------------------------
    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    Experiment().run()