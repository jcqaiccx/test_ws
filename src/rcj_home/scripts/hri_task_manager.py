#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import difflib
import rospy
from dataclasses import dataclass
from enum import Enum
from std_msgs.msg import String
from robot_voice.srv import RecognizeVoice


class HRIState(Enum):
    S0_INIT = "S0_INIT"
    S1_ASK_INFO = "S1_ASK_INFO"
    S2_OUTPUT_INFO = "S2_OUTPUT_INFO"
    TASK_DONE = "TASK_DONE"


@dataclass
class GuestInfo:
    guest_id: int
    name: str = ""
    drink: str = ""
    seat_id: int = -1


class HRITaskManager:
    def __init__(self):
        rospy.init_node("hri_task_manager", anonymous=False)

        self.state = HRIState.S0_INIT
        self.guests = []
        self.seat_status = [False, False, False]  # False means free
        self.current_guest = None

        self.tts_wait_timeout = rospy.get_param("~tts_wait_timeout", 20.0)
        self.tts_pub = rospy.Publisher("/tts_text", String, queue_size=10)
        self.state_pub = rospy.Publisher("/task/state", String, queue_size=10)
        self.tts_done_sub = rospy.Subscriber("/tts_done", String, self.tts_done_callback, queue_size=10)

        self.waiting_tts_text = None
        self.tts_done_ok = False

        self.voice_timeout = rospy.get_param("~voice_timeout", 12)
        self.loop_hz = rospy.get_param("~loop_hz", 2)

        # Name and drink dictionaries for constraint + typo correction.
        self.name_candidates = [
            "adam", "alex", "alice", "amelia", "andrew", "anna", "ben", "charlie", "chloe",
            "daniel", "david", "ella", "emily", "emma", "ethan", "eva", "grace", "harry",
            "henry", "isabella", "jack", "james", "jason", "jenny", "john", "julia", "kate",
            "kevin", "leo", "liam", "lily", "lucas", "lucy", "mia", "michael", "nancy",
            "olivia", "oscar", "peter", "rose", "ryan", "sarah", "sophia", "thomas", "tom",
            "victor", "william", "zoe"
        ]

        self.drink_alias_map = {
            "water": ["water", "mineral water", "sparkling water"],
            "tea": ["tea", "black tea", "green tea", "milk tea"],
            "coffee": ["coffee", "latte", "cappuccino", "espresso", "americano"],
            "cola": ["cola", "coke", "coca cola", "coca-cola", "pepsi"],
            "juice": ["juice", "orange juice", "apple juice", "grape juice", "lemon juice"],
            "milk": ["milk", "hot milk", "cold milk"],
            "lemonade": ["lemonade"],
            "soda": ["soda", "soft drink"],
            "beer": ["beer"],
            "wine": ["wine", "red wine", "white wine"],
            "whisky": ["whisky", "whiskey"],
        }

        self.drink_alias_flat = []
        for aliases in self.drink_alias_map.values():
            self.drink_alias_flat.extend(aliases)

    def _normalize_name(self, raw_name: str):
        if not raw_name:
            return ""
        token = raw_name.strip().lower().split()[0]
        if token in self.name_candidates:
            return token.title()

        close = difflib.get_close_matches(token, self.name_candidates, n=1, cutoff=0.72)
        if close:
            return close[0].title()

        return token.title()

    def _normalize_drink(self, raw_drink: str, full_text: str):
        raw = (raw_drink or "").strip().lower()
        text = (full_text or "").strip().lower()

        # Prefer direct alias match in full sentence to avoid partial extraction errors.
        all_aliases = sorted(self.drink_alias_flat, key=len, reverse=True)
        for alias in all_aliases:
            if re.search(r"\b" + re.escape(alias) + r"\b", text):
                for canonical, aliases in self.drink_alias_map.items():
                    if alias in aliases:
                        return canonical.title()

        # Fuzzy match on extracted drink segment.
        if raw:
            close = difflib.get_close_matches(raw, all_aliases, n=1, cutoff=0.65)
            if close:
                alias = close[0]
                for canonical, aliases in self.drink_alias_map.items():
                    if alias in aliases:
                        return canonical.title()

            tokens = raw.split()
            for n in [3, 2, 1]:
                for i in range(0, max(0, len(tokens) - n + 1)):
                    phrase = " ".join(tokens[i:i + n])
                    close = difflib.get_close_matches(phrase, all_aliases, n=1, cutoff=0.68)
                    if close:
                        alias = close[0]
                        for canonical, aliases in self.drink_alias_map.items():
                            if alias in aliases:
                                return canonical.title()

        # Fallback: keep first two words to avoid excessively long noisy phrase.
        if raw:
            return " ".join(raw.split()[:2]).title()
        return ""

    def tts_done_callback(self, msg: String):
        if self.waiting_tts_text is None:
            return
        if msg.data == self.waiting_tts_text:
            self.tts_done_ok = True

    def say(self, text: str, wait_done: bool = True):
        self.waiting_tts_text = text
        self.tts_done_ok = False
        self.tts_pub.publish(String(data=text))
        rospy.loginfo("TTS: %s", text)

        if not wait_done:
            return

        # Wait until tts_subscribe publishes /tts_done for the same text.
        start = rospy.Time.now()
        timeout_sec = self.tts_wait_timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.tts_done_ok:
                break
            if (rospy.Time.now() - start).to_sec() > timeout_sec:
                rospy.logwarn("Timeout waiting for /tts_done, continue state flow")
                break
            rate.sleep()

        self.waiting_tts_text = None

    def publish_state(self):
        self.state_pub.publish(String(data=self.state.value))

    def call_asr(self):
        rospy.wait_for_service("/voice/recognize", timeout=5.0)
        recognize = rospy.ServiceProxy("/voice/recognize", RecognizeVoice)
        resp = recognize(self.voice_timeout)
        return resp.text.strip()

    def parse_name_drink(self, text: str):
        # Rule templates + dictionary constraints for better robustness.
        name = ""
        drink = ""

        # Remove punctuation and squeeze spaces.
        clean_text = re.sub(r'[^\w\s]', ' ', text.lower())
        clean_text = re.sub(r'\s+', ' ', clean_text).strip()

        patterns_name = [
            r"my name is\s+([a-z]+)",
            r"this is\s+([a-z]+)",
            r"you can call me\s+([a-z]+)",
            r"people call me\s+([a-z]+)",
            r"i am called\s+([a-z]+)",
            r"i am\s+([a-z]+)",
            r"i am mr\s+([a-z]+)",
            r"i am mrs\s+([a-z]+)",
            r"i am miss\s+([a-z]+)",
            r"im\s+mr\s+([a-z]+)",
            r"im\s+mrs\s+([a-z]+)",
            r"im\s+miss\s+([a-z]+)",
            r"im\s+([a-z]+)",
            r"name is\s+([a-z]+)"
        ]
        patterns_drink = [
            r"favorite drink is\s+([a-z\s]+)",
            r"my favorite drink is\s+([a-z\s]+)",
            r"my favourite drink is\s+([a-z\s]+)",
            r"my favorite is\s+([a-z\s]+)",
            r"my favourite is\s+([a-z\s]+)",
            r"i like\s+([a-z\s]+)",
            r"i really like\s+([a-z\s]+)",
            r"i love\s+([a-z\s]+)",
            r"i usually drink\s+([a-z\s]+)",
            r"i prefer\s+([a-z\s]+)",
            r"i want\s+([a-z\s]+)",
            r"i want some\s+([a-z\s]+)",
            r"give me\s+([a-z\s]+)",
            r"drink is\s+([a-z\s]+)",
            r"for drink\s+([a-z\s]+)"
        ]

        raw_name = ""
        raw_drink = ""

        for p in patterns_name:
            m = re.search(p, clean_text)
            if m:
                raw_name = m.group(1).strip()
                break

        for p in patterns_drink:
            m = re.search(p, clean_text)
            if m:
                raw_drink = m.group(1).strip()
                break

        name = self._normalize_name(raw_name)
        drink = self._normalize_drink(raw_drink, clean_text)

        rospy.loginfo("Parsed guest info => name_raw='%s', drink_raw='%s', name='%s', drink='%s'",
                      raw_name, raw_drink, name, drink)
        return name, drink

    def allocate_seat(self):
        for i, occupied in enumerate(self.seat_status):
            if not occupied:
                self.seat_status[i] = True
                return i
        return -1

    def run(self):
        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown() and self.state != HRIState.TASK_DONE:
            self.publish_state()

            if self.state == HRIState.S0_INIT:
                self.say("System initialized. Ready to ask for guest information.")
                self.state = HRIState.S1_ASK_INFO

            elif self.state == HRIState.S1_ASK_INFO:
                self.current_guest = GuestInfo(guest_id=1)
                self.say("Hello, welcome. What is your name and favorite drink?")
                # Give audio output a short guard interval before opening microphone.
                rospy.sleep(0.6)
                try:
                    text = self.call_asr()
                    rospy.loginfo("ASR text: %s", text)
                    if not text:
                        self.say("Sorry, I did not catch that. Please say your name and favorite drink again.")
                        text = self.call_asr()
                        rospy.loginfo("ASR retry text: %s", text)
                    name, drink = self.parse_name_drink(text)
                    self.current_guest.name = name if name else f"Guest{self.current_guest.guest_id}"
                    self.current_guest.drink = drink if drink else "Unknown Drink"
                except Exception as e:
                    rospy.logwarn("ASR failed: %s", e)
                    self.current_guest.name = f"Guest{self.current_guest.guest_id}"
                    self.current_guest.drink = "Unknown Drink"
                
                self.state = HRIState.S2_OUTPUT_INFO

            elif self.state == HRIState.S2_OUTPUT_INFO:
                # Output the extracted information correctly
                info_text = f"Got it. Your name is {self.current_guest.name}, and your favorite drink is {self.current_guest.drink}."
                rospy.loginfo(info_text)
                self.say(info_text)
                
                self.state = HRIState.TASK_DONE

            rate.sleep()

        self.publish_state()
        self.say("Receptionist task is completed.")


if __name__ == "__main__":
    try:
        manager = HRITaskManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
