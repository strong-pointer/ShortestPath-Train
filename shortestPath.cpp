#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <queue>
#include <cmath>
using namespace std; 

struct Pixel {
    unsigned char r, g, b;
};
// "Node" Struct
struct Coordinate {
    // Current x and y
    int x, y;
    // Previous x and y
    int px, py;
    // Bit flag to represent which passengers have been picked up
    int passengers;
    bool operator==(const Coordinate& o) {
        return x == o.x && y == o.y && px == o.px && py == o.py && passengers == o.passengers;
    }
};

bool operator<(const Coordinate&o, const Coordinate&z) {
    if (o.passengers == z.passengers) {
        if (o.py == z.py) {
            if (o.px == z.px) {
                if (o.y == z.y) return o.x < z.x;
                return o.y < z.y;
            }
            return o.px < z.px;
        }
        return o.py < z.py;
    }
    return o.passengers < z.passengers;
}

bool operator!=(const Coordinate&o, const Coordinate&z) {
    return o.passengers != z.passengers || o.py != z.py || o.px != z.px || o.y != z.y || o.x != z.x;
}

// nth bit(s)
const int PASSENGER1=0, PASSENGER2=1, PASSENGER3=2, PASSENGER4=3;
char ppmName[50]; 
Coordinate start, goal, passenger1, passenger2, passenger3, passenger4, newGoal;
int width, height, maxPixSum;
double minEuc, maxEuc, maxDeg, D, maxRad, D2, min2, max2;
Pixel **Image;
FILE* out;
// Dijkstra's variables
map<Coordinate, double> dist;
map<Coordinate, Coordinate> pred;
double inf = 999999.0;
typedef pair<double,Coordinate> pin;

// Return nth bit of x
bool get_bit(int x, int n)
{
    return (x >> n) & 1;
}

// 0 0 0 0 
// 0 0 0 1       x(0000) | (1<<0)
// 1 0 0 1       x(0001) | 1000 
int set_bit(int x, int name) 
{
    return x | (1<<name);
}

// Reading in the ppm file, fills in **Image
void read_in_image() 
{
    FILE* fp = fopen(ppmName, "r");
    fscanf(fp, "P6\n%d %d\n255%*c", &width, &height);
    Image = new Pixel*[height];
    Image[0] = new Pixel[height*width];
    for (int i = 1; i < height; i++) {
        Image[i] = Image[i-1] + width;
    }
    fread(Image[0], width*height, sizeof(Pixel), fp);
    fclose(fp);
}

// Writes whatever **Image has saved to its pixels
void write_to_image() 
{
    FILE* out = fopen("path.ppm", "w");
    fprintf(out, "P6\n%d %d\n255\n", width, height);
    fwrite(Image[0], width*height, sizeof(Pixel), out);
}

// Returns the sum of a coordinate point on **Image's r, g, and b values
int sum_of_color(const Coordinate& c) {
    return Image[c.y][c.x].r + Image[c.y][c.x].g + Image[c.y][c.x].b;
}

// Math function(s):
double euclidian_dist(const Coordinate& x, const Coordinate& y) 
{
    // sqrt((x2-x1)^2 + (y2-y1)^2)
    return sqrt(((y.x - x.x)*(y.x - x.x) * 1.0 + (y.y - x.y)*(y.y - x.y)) * 1.0);
}

double euclidian_dist_NoSqrt(const Coordinate& x, const Coordinate& y) 
{
    // (x2-x1)^2 + (y2-y1)^2
    return ((y.x - x.x)*(y.x - x.x) * 1.0 + (y.y - x.y)*(y.y - x.y)) * 1.0;
}

bool valid_angle(double temp1x, double temp1y, const Coordinate& currC, const Coordinate& nextC) 
{
    double temp2x, temp2y;

    temp2x = nextC.x - currC.x;
    temp2y = nextC.y - currC.y;
    double mag2 = sqrt((temp2x*temp2x) + (temp2y*temp2y));

    temp2x /= mag2;
    temp2y /= mag2;

    double dot = ((temp1x * temp2x) + (temp1y * temp2y));
    return dot > maxRad;
}

// Finding neighbors for a singular point
vector<Coordinate> get_nbrs(const Coordinate& point) 
{
    vector<Coordinate> nbrVec;
    int prev_x = point.x;
    int prev_y = point.y;
    int p, q;
    Coordinate previousCoord = { point.px, point.py, -1, -1, -1 };
    // First (mathematical) vector for valid_angle() variables:
    int counter = 0;
    double temp1x, temp1y;
    
    // Start at 0 if the max distance is out of bounds
    for (p = point.y - maxEuc < 0 ? 0 : point.y - maxEuc; p <= point.y + maxEuc && p < height; p++) {
        for (q = point.x - maxEuc < 0 ? 0 : point.x - maxEuc; q <= point.x + maxEuc && q < width; q++) {
            Coordinate x = { q, p, prev_x, prev_y, 0 };
            // Constraints: color threshold, min and max distance requirement, and then finally angle requirement
            if (sum_of_color(x) <= maxPixSum) {
                double distance = euclidian_dist_NoSqrt(point, x);
                if (distance >= min2 && distance <= max2) {
                    // Calculating the first vector used in valid_angle() once per call to get_nbrs()
                    if ((point.px != -1 && point.py != 1) && counter == 0) {
                        temp1x = point.x - previousCoord.x;
                        temp1y = point.y - previousCoord.y;
                        double mag1 = sqrt((temp1x*temp1x) + (temp1y*temp1y));

                        temp1x /= mag1;
                        temp1y /= mag1;
                        counter++;
                    }
                    // Must be a valid angle (or the starting point(which have previous point(s) set to -1))
                    if ((point.px == -1 && point.py == -1) || valid_angle(temp1x, temp1y, point, x)) {
                        // Check if people to pick up are nearby, adds them if they are
                        if (euclidian_dist_NoSqrt(x, passenger1) <= D2) x.passengers = set_bit(x.passengers, PASSENGER1);
                        else if (euclidian_dist_NoSqrt(x, passenger2) <= D2) x.passengers = set_bit(x.passengers, PASSENGER2);
                        else if (euclidian_dist_NoSqrt(x, passenger3) <= D2) x.passengers = set_bit(x.passengers, PASSENGER3);
                        else if (euclidian_dist_NoSqrt(x, passenger4) <= D2) x.passengers = set_bit(x.passengers, PASSENGER4);

                        nbrVec.push_back(x);
                    }
                }
            }
        }
    }
    return nbrVec;
}

// Dijkstra's algo to find the optimal path
void visit(const Coordinate& source, const Coordinate& dest) 
{
    dist[source] = 0.0;
    priority_queue<pin, vector<pin>, greater<pin>> to_visit; 
    to_visit.push(make_pair(0.0, source));
    pred[source] = source;

    while (!to_visit.empty()) {
        Coordinate x = to_visit.top().second;
        to_visit.pop();

        double dis_from_dest = euclidian_dist(dest, x);
        if (dis_from_dest <= D && (x.passengers == 0b1011 || x.passengers == 0b0111)) {
            cout << "Found path!";
            newGoal = x;
            return;
        }

        for (Coordinate& n : get_nbrs(x)) {
            // Update n's passengers (if they dont already have the same passengers as x)
            n.passengers |= x.passengers;
            if (dist.find(n) == dist.end()) dist[n] = inf;
            double weight = euclidian_dist(x, n);

            if (dist[x] + weight < dist[n]) {
	            dist[n] = dist[x] + weight;
	            pred[n] = x;
	            to_visit.push(make_pair(dist[n], n));
            }
        }
    }
}

void print_path(const Coordinate& dest) 
{
    if (dest != pred[dest]) print_path(pred[dest]);
    // Write coordinates to txt
    fprintf(out, "%d %d\n", dest.x, dest.y);
    // Write path to image in 7x7 cube
    int p, q;
    p = dest.y - 3 < 0 ? 0 : dest.y - 3;
    for (p; p <= dest.y + 3 && p < height; p++) {
        for (q = dest.x - 3 < 0 ? 0 : dest.x - 3; q <= dest.x + 3 && q < width; q++) {
            Image[p][q] = { 245, 102, 0 };
        }
    }
}

void print_cubes(const Coordinate& point, const Pixel& color) 
{
    int p, q;
    p = point.y - 6 < 0 ? 0 : point.y - 6;
    for (p; p <= point.y + 6 && p < height; p++) {
        for (q = point.x - 6 < 0 ? 0 : point.x - 6; q <= point.x + 6 && q < width; q++) {
            Image[p][q] = color;
        }
    }
}

int main(int argc, char *argv[]) 
{
    // Reading in config file items from command line argument
    ifstream config(argv[1]);
    config >> ppmName;
    config >> start.x >> start.y;
    start.passengers = 0;
    start.px = -1;
    start.py = -1;
    goal.passengers = 0;
    goal.px = 0;
    goal.py = 0;
    config >> goal.x >> goal.y;
    config >> passenger1.x >> passenger1.y;
    config >> passenger2.x >> passenger2.y;
    config >> passenger3.x >> passenger3.y;
    config >> passenger4.x >> passenger4.y;
    config >> maxPixSum;
    config >> minEuc;
    config >> maxEuc;
    config >> maxDeg;
    config >> D;
    config.close();
    read_in_image();
    
    // Pre-compute max radians for sake of running time, used in valid_angle()
    maxRad = cos(maxDeg*(M_PI / 180.0));
    // Squared global variables (reducing time spent on sqrt() in get_nbrs())
    D2 = D * D;
    min2 = minEuc * minEuc;
    max2 = maxEuc * maxEuc;
    
    // Finding the shortest path given the constraints
    visit(start, goal);

    // Color the start, goal, and passenger point(s)
    Pixel color;
    color = { 0, 255, 0 };
    print_cubes(start, color);
    color = { 255, 0, 0 };
    print_cubes(goal, color);
    color = { 0, 0, 255 };
    print_cubes(passenger2, color);
    print_cubes(passenger1, color);
    print_cubes(passenger3, color);
    print_cubes(passenger4, color);

    // Opening the path.txt file to write coordinates of path
    out = fopen("path.txt", "w");
    print_path(newGoal);
    write_to_image();
    fclose(out);
}