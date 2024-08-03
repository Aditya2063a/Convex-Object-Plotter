import socket
import threading
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Polygon
import numpy as np

from scipy.spatial import ConvexHull


import math
import matplotlib.pyplot as plt

# Function to calculate the polar angle between two points
def remove_nearby_coordinates(x, y):
    result_x = [x[0]]  # Keep the first available coordinate
    result_y = [y[0]]

    i = 1
    while i < len(x):
        curr_x = x[i]
        curr_y = y[i]

        # Check if current coordinate is more than 9 units away from all previously selected coordinates
        is_far_away = True
        for j in range(len(result_x)):
            dist = math.sqrt((curr_x - result_x[j]) ** 2 + (curr_y - result_y[j]) ** 2)
            if dist < 9:
                is_far_away = False
                break

        if is_far_away:
            # If current coordinate is far away, add it to the result
            result_x.append(curr_x)
            result_y.append(curr_y)

        i += 1

    return result_x, result_y

def polar_angle(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    return math.atan2(dy, dx)

# Function to check if three points make a right turn
def right_turn(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    return (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1) < 0

# Function to find the convex hull using Graham's scan algorithm
'''def convex_hull(x, y):
    points = list(zip(x, y))

    # Sort points based on polar angle
    lowest = min(points, key=lambda p: (p[1], p[0]))
    sorted_points = sorted(points, key=lambda p: polar_angle(lowest, p))

    # Build the convex hull
    hull = [sorted_points[0], sorted_points[1]]
    for i in range(2, len(sorted_points)):
        while len(hull) > 1 and right_turn(hull[-2], hull[-1], sorted_points[i]):
            hull.pop()
        hull.append(sorted_points[i])

    return hull'''



def create_convex_polygon(x_coords, y_coords, distance_threshold):
    # Combine x and y coordinates into a single array of points
    points = np.column_stack((x_coords, y_coords))

    # Remove repetitions within the distance threshold
    unique_points = []
    for point in points:
        if all(np.linalg.norm(point - unique_point) > distance_threshold for unique_point in unique_points):
            unique_points.append(point)

    # Calculate the Convex Hull
    hull = ConvexHull(unique_points)

    # Get the coordinates of the hull's vertices
    hull_vertices = unique_points[hull.vertices]

    return hull_vertices

# Example usage







def perimeterfind():
    perimeter=0

    for i in range(len(x)):
        if i == len(x)-1:
            perimeter += ((x[i]-x[0])**2+(y[i]-y[0])**2)**0.5
        else:
            perimeter+=((x[i]-x[i+1])**2+(y[i]-y[i+1])**2)**0.5
    return perimeter


def areafind():
    n = len(x)
    area = 0

    for i in range(n):
        x1, y1 = x[i], y[i]
        x2, y2 = x[(i + 1) % n], y[(i + 1) % n]  # Wrap around to the first vertex

        area += (x1 * y2) - (x2 * y1)

    area = abs(area) / 2

    return area







nc=0

xx=[]
yy=[]

cx=[]
cy=[]

HOST = socket.gethostbyname(socket.gethostname())   # Use an empty string to accept connections on all available interfaces
PORT = 8000  # Replace with the same port number used in the ESP32 code

def start_server():
    def run_server():
        global nc,x,y,xx,yy

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

            s.bind((HOST, PORT))
            print('Server started')
            print(socket.gethostbyname(socket.gethostname()) )

            # Listen 
            s.listen(1)
            print('Waiting for connection...')

            conn, addr = s.accept()
            print('Connected by', addr)

            conn.sendall(b'start')
            try:
                while True:

                    data = conn.recv(1024).decode('utf-8')
                    
                    print(data)


                    if data == "NC":
                        nc=1

                        window.destroy()
                        break


                    
                    elif data[0]=="A":
                        data3=data[2:]
                        data3=data3.strip().split()


                        for i in range(len(data3)):
                            if i%2==0:
                                cx.append(float(data3[i]))
                            else:
                                cy.append(float(data3[i]))

                        print(cx,cy)


                    elif data == "STOP":
                        window.destroy()
                        break
                    else:
                        data1=data.split(',')

                        xx.append(float(data1[0]))
                        yy.append(float(data1[1]))
                    


                    if not data:
                        break

                # Print the received data
                #print('Received:', x,y)
                text_area.insert(tk.END, data + '\n')
            except:
                window.destroy()

            print('Connection closed')


    server_thread = threading.Thread(target=run_server)
    server_thread.start()


# Create the main window
window = tk.Tk()
window.title('Data Receiver')

# Create the start button
start_button = tk.Button(window, text='Start', command=start_server)
start_button.pack(pady=10)

# Create the text area to display the received data
text_area = ScrolledText(window, height=10, width=40)
text_area.pack(padx=10, pady=5)

# Start the GUI event loop
window.mainloop()



if nc == 0:
    plt.scatter(xx,yy)
    plt.title("Convex Shape")
    plt.xlabel("cm")
    plt.ylabel("cm")
    plt.show()
    

    '''
    distance_threshold = 9

    convex_polygon = create_convex_polygon(x, y, distance_threshold)

    # Print the resulting convex polygon
    for i, vertex in enumerate(convex_polygon):
        print(f"Corner {i+1}: ({vertex[0]}, {vertex[1]})")'''


    '''
    hull = convex_hull(xx, yy)

    # Separate x and y coordinates for plotting
    hull_x, hull_y = zip(*hull)

    # Plot the points and the convex hull polygon
    plt.figure()
    plt.plot(xx, yy, 'bo', label='Points')
    plt.plot(hull_x, hull_y, 'r-', label='Convex Hull')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Convex Hull')
    plt.legend()
    plt.grid(True)
    plt.show()'''

    x, y = remove_nearby_coordinates(cx, cy)

    print(x,y)





    perimeter=perimeterfind()

    area=areafind()



    # Create the GUI window
    window1 = tk.Tk()
    window1.title("Convex Polygon")

    # Create a frame for the polygon plot
    plot_frame = tk.Frame(window1)
    plot_frame.pack(side=tk.TOP, pady=10)

    # Create a figure and plot the polygon
    fig = Figure(figsize=(5, 5))
    canvas = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas.get_tk_widget().pack()

    ax = fig.add_subplot(111)
    polygon = Polygon(np.column_stack((x, y)), edgecolor='black', facecolor='blue', alpha=0.5)
    ax.add_patch(polygon)
    ax.autoscale_view()

    # Create labels for name, perimeter, and area
    name_label = tk.Label(window1, text="Polygon Name: Convex Polygon", font=("Arial", 14))
    name_label.pack()

    perimeter_label = tk.Label(window1, text=f"Perimeter: {perimeter}", font=("Arial", 12))
    perimeter_label.pack()

    area_label = tk.Label(window1, text=f"Area: {area}", font=("Arial", 12))
    area_label.pack()

    # Run the GUI main loop
    window1.mainloop()

    



    


else:
    print("NOT A CONVEX SHAPE")


