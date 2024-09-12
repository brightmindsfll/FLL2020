print("WELCOME TO BM MOTIPLAY®")
print("Are you available?")
print("1. Yes")
print("2. No")

ans=True
while ans:

    ans=input("What would you like to do? ")
    if ans=="1": 
        print ("Choose a workout duration")
        print ("1. 30 minutes")
        print ("2. 1 hour")
        print ("3. 2 hours")
        print ("4. custom")
        
        app_choice = True
        
        while app_choice:
            app_choice = app_choice=input("What would you like to do? ")
            if app_choice == "1":
                print("What type of workout would you like to do? ")
                print("1. Walking")
                print("2. Jogging/Running")
                print("3. Yoga/Meditation")
                print("4. Dance/Zumba")
                print("5. Sports")

                workout = True
                while workout:
                    workout = workout=input("What would you like to do? ")
                    if workout == "1":
                        print("How do you want to work out?")
                        print("1. Individually")
                        print("2. With a partner")
                        print("3. With a group")                    

                        friends = True
                        while friends:
                            friends = friends=input("What would you like to do? ")
                            if friends == "1":
                                print("Do you want to train with a trainer/coach?")
                                print("1. Yes")
                                print("2. No")

                                trainer = True

                                while trainer:
                                    trainer = trainer=input("What would you like to do? ")
                                    if trainer == "1":
                                        print("trainers and coaches that are available")
                                        print("*insert trainer names*")
                                        break
                                    elif trainer == "2":
                                        print("Here are links for motivational music:")
                                        print("1. https://open.spotify.com/track/1yvMUkIOTeUNtNWlWRgANS?si=IjR15cSCQ1Kjc-9GbGNAJw")
                                        print("2. https://open.spotify.com/track/5C1PksOQ542mrdjr3BX86P?si=rADQlc92Qq2f6T995w2MhQ")

                            elif friends == "2":
                                print("Available friends in the same boat")
                                print("1. Chloe Stevenson")
                                print("2. Larry Stylinson")
                            elif friends == "3":
                                break

                    elif workout == "5":
                        print("What sport do you want to play?")
                        print("1. Tennis")
                        print("2. Soccer")
                        print("3. Swimming")
                        print("3. Basketball")

                        sport = True

                        while sport:
                            sport = sport=input("What would you like to do? ")
                            if sport == "1":
                                print("Where do you want to play tennis?")
                                print("1. Cary Tennis Park")
                                print("2. Annie Jones Park")
                                print("3. Apex Community Park")
                                print("4. Kentwood Park")
                                print("5. Millbrook Tennis Center")

                            elif sport == "2":
                                print("Where do you want to play soccer?")
                                print("1. North Cary Park: Soccer Field")
                                print("2. WakeMed Soccer Park")
                                print("3. C.M. Herndon Park")
                                print("4. Smith Soccer Field")

                            elif sport == "3":
                                print("Where do you want to swim?")
                                print("1. Optimist Pool")
                                print("2. Westover Swimming Pool")
                                print("3. LIFETIME Fitness Center")
                                print("4. Homestead Acquatic Center")

                            elif sport == "4":
                                print("Where do you want to play basketball?")
                                print("1. Cary Tennis Park")
                                print("2. Annie Jones Park")
                                print("3. Apex Community Park")
                                print("4. Kentwood Park")
                                print("5. Millbrook Tennis Center")




                               


    

