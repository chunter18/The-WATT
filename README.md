# The-WATT
Arduino based Sous-Vide cooker. (Pronounced soo-vee)

# Features
Low cost, high wattage, easy to operate Sous-Vide with all the features of a $400 cooker, with the added benefit of having some fun building it.

# Rationale
As of writing this, it is the summer after my sophmore year of college studying electrical engineering. What that means to me is that I will be living off campus in an apartment, and losing access to a school meal plan and gaining a strict meal budget. Cooking and eating good food is a hobby of mine, and good cooking reqires good equiptment. A Sous-Vide cooker makes a great addition to my arsenal for a number of reasons -  

1. Ease of cooking with an engineering students schedule. Toss almost any meat/veggie/combo in the cooker and leave it for hours at a time. When hungry, sear meat in a pan for 45 seconds and dinner is served. 

2. Quality of food produced. The Sous-Vide has similar benefits to a crockpot/slow cooker, but with much finer grain control which allows for higher quality end products - especially with meat. A perfect medium rare cut of beef is a snap with a Sous-Vide, and no matter how long you leave it, you wont lose the perfect internal temperature. A crockpot cant keep up.

3. Most importantly, the savings! Eating high quality meat is expensive, a habit that reflects poorly on a savings account. With a Sous-Vide, I can buy large cuts of tougher, flavorful meat (i.e. tri-tip, hanger steak) and turn them into meals that are cheap and taste like a million bucks.

So all in all, a Sous-Vide makes perfect sense. The only problem is, a high wattage, good quality Sous-Vide can cost $150 all the way to $400 dollars (since my original research there have been lower cost units have come out with lower wattages, but mine still beats those in all but looks) . With some extra time on my hands during the summer, I built the WATT.  


# Parts List:
| Item | Price (USD) | Link |
|---|---|---|
| Arduino Nano | $3.00 | | 
| Camco 1000W Heater element | $13.55 | |
| Solid State Relay| $9.95 | https://www.sparkfun.com/products/13015 |
| Gorilla brand Silicone Sealant | $4.84 | |
| Circulation Pump | $8.99 | |
| Adafruit LCD Shield Kit | $19.95 | https://www.adafruit.com/product/772 |
| Waterproof DS18B20 temperature sensor | $9.95 | https://www.adafruit.com/product/381 |
| 1" Female Threaded Copper Coupling | $5-9 | |
| AC Cable with plug | $0 (Chop one off an old appliance) | N/A |

Tools/Equiptment
-USB A to mini-B to program arduino

Optional Items
Buzzer

Total cost:
$75.23

Many of the items here that are electronic in nature I had in my possesion already, making the cost lower. Substitutions can be made for most of these items for parts on hand in another build. 

One thing that I didnt want to cheap out on was any part that handeled mains alternating current. As this is was a DIY project, and not thouroughly checked out by anybody but me, I decided to be extra safe. The relay, spade connectors, and wire all are rated for more current than they should ever reasonably be pulling. 

# Pictures:

# Final Thoughts
This build is still unrefined. While it has full functionality and can cook some delicious food, it is a pain to set up and looks very messy (which makes it slightly unsafe). The build definitely needs some polishing. In the future, I'd like to combine all the components into a single unit, along the lines of what was done in this photo: 

![Alt text](sous_vide_example.jpg?raw=true)

This would potientially mean designing a custom pcb which I would love to do. I would also love to move past a 16x2 LCD. I did the best I could with so little space, but the menu system could be cleaned up significantly. 

Though the WATT isnt exactly like I want at this point, it was still quite a bit of fun to build and will continue to pay dividends for quite some time!
