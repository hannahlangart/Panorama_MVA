// Imagine++ project
// Project:  Panorama
// Author:   Pascal Monasse
// Date:     2013/10/08

// Hannah Langart

#include <Imagine/Graphics.h>
#include <Imagine/Images.h>
#include <Imagine/LinAlg.h>
#include <vector>
#include <sstream>

using namespace Imagine;
using namespace std;

// Record clicks in two images, until right button click
void getClicks(Window w1, Window w2,
               vector<IntPoint2>& pts1, vector<IntPoint2>& pts2) {

  int button = 0;
  int pairs = 0;
  IntPoint2 clicked_pts1;
  IntPoint2 clicked_pts2;

  cout << "Select at least 4 pairs of matching points in the two images"
       << endl;

  // User selects a point in one image, then its correspondence in second image
  while (button != 3) { // until right click
    setActiveWindow(w2);
    showWindow(w2);
    cout << "Select a keypoint in the first image" << endl;
    getMouse(clicked_pts2);
    drawCircle(clicked_pts2, 3, RED, 2);
    pts2.push_back(clicked_pts2);

    setActiveWindow(w1);
    showWindow(w1);
    cout << "Select the corresponding point in the second image... Use the right click to end the operation"<< endl;
    button = getMouse(clicked_pts1);
    drawCircle(clicked_pts1, 3, RED, 2);
    pts1.push_back(clicked_pts1);

    pairs++;
  }

  cout << "You have selected " + to_string(pairs) + " pairs of matching points." << endl;
}

// Return homography compatible with point matches
Matrix<float> getHomography(const vector<IntPoint2>& pts1,
                            const vector<IntPoint2>& pts2) {
    size_t n = min(pts1.size(), pts2.size());
    if(n<4) {
        cout << "Not enough correspondences: " << n << endl;
        return Matrix<float>::Identity(3);
    }

    Matrix<double> A(2*n,8);
    Vector<double> B(2*n);

    IntPoint2 match1;
    IntPoint2 match2;

    // Solving the linear equation according to A_i matrix entries
    for(int i=0; i<n; i++) {

        int x1;
        int y1;
        match1=pts1[i];
        x1=match1[0];
        y1=match1[1];

        int x2;
        int y2;
        match2=pts2[i];
        x2=match2[0];
        y2=match2[1];

        A(2 * i, 0) = x1;
        A(2 * i, 1) = y1;
        A(2 * i, 2) = 1;
        A(2 * i, 3) = 0;
        A(2 * i, 4) = 0;
        A(2 * i, 5) = 0;
        A(2 * i, 6) = -x1 * x2;
        A(2 * i, 7) = -x2 * y1;

        A(2 * i + 1, 0) = 0;
        A(2 * i + 1, 1) = 0;
        A(2 * i + 1, 2) = 0;
        A(2 * i + 1, 3) = x1;
        A(2 * i + 1, 4) = y1;
        A(2 * i + 1, 5) = 1;
        A(2 * i + 1, 6) = -x1 * y2;
        A(2 * i + 1, 7) = -y2 * y1;

        B[2 * i] = x2;
        B[2 * i + 1] = y2;
        }

    B = linSolve(A, B);
    Matrix<float> H(3, 3);
    H(0,0)=B[0]; H(0,1)=B[1]; H(0,2)=B[2];
    H(1,0)=B[3]; H(1,1)=B[4]; H(1,2)=B[5];
    H(2,0)=B[6]; H(2,1)=B[7]; H(2,2)=1;

    // Sanity check
    for(size_t i=0; i<n; i++) {
        float v1[]={(float)pts1[i].x(), (float)pts1[i].y(), 1.0f};
        float v2[]={(float)pts2[i].x(), (float)pts2[i].y(), 1.0f};
        Vector<float> x1(v1,3);
        Vector<float> x2(v2,3);
        x1 = H*x1;
        cout << x1[1]*x2[2]-x1[2]*x2[1] << ' '
             << x1[2]*x2[0]-x1[0]*x2[2] << ' '
             << x1[0]*x2[1]-x1[1]*x2[0] << endl;
    }
    return H;
}

// Grow rectangle of corners (x0,y0) and (x1,y1) to include (x,y)
void growTo(float& x0, float& y0, float& x1, float& y1, float x, float y) {
    if(x<x0) x0=x;
    if(x>x1) x1=x;
    if(y<y0) y0=y;
    if(y>y1) y1=y;    
}

// Panorama construction
void panorama(const Image<Color,2>& I1, const Image<Color,2>& I2,
              Matrix<float> H) {
    Vector<float> v(3);
    float x0=0, y0=0, x1=I2.width(), y1=I2.height();

    v[0]=0; v[1]=0; v[2]=1;
    v=H*v; v/=v[2];
    growTo(x0, y0, x1, y1, v[0], v[1]);

    v[0]=I1.width(); v[1]=0; v[2]=1;
    v=H*v; v/=v[2];
    growTo(x0, y0, x1, y1, v[0], v[1]);

    v[0]=I1.width(); v[1]=I1.height(); v[2]=1;
    v=H*v; v/=v[2];
    growTo(x0, y0, x1, y1, v[0], v[1]);

    v[0]=0; v[1]=I1.height(); v[2]=1;
    v=H*v; v/=v[2];
    growTo(x0, y0, x1, y1, v[0], v[1]);

    cout << "x0 x1 y0 y1=" << x0 << ' ' << x1 << ' ' << y0 << ' ' << y1<<endl;

    Image<Color> I(int(x1-x0), int(y1-y0));
    setActiveWindow( openWindow(I.width(), I.height()) );
    I.fill(WHITE);

    //Color of the corresponding pixels in image 1 and image 2
    Color col1;
    Color col2;

    // True if the color pixel lies in the corresponding image
    bool img1ok;
    bool img2ok;

    for (int x = int(x0); x < int(x1); x++) {

      for (int y = int(y0); y < int(y1); y++) {

        img1ok = false;
        img2ok = false;

        // We check for the pixel in image 2
        if ((0 <= x && x < I2.width()) && (0 <= y && y < I2.height())) {
          col2 = I2(x, y);
          img2ok = true;
        }

        // We check for the pixel in image 1 by using the inverse homography matrix
        v[0] = x;
        v[1] = y;
        v[2] = 1;
        v = inverse(H) * v;
        v /= v[2];
        int x_p = int(v[0]);
        int y_p = int(v[1]);

        if ((0 <= x_p && x_p < I1.width()) && (0 <= y_p && y_p < I1.height())) {
          col1 = I1(x_p, y_p);
          img1ok = true;
        }

        // If the pixel lies within both images, we take the average of colors at corresponding pixels in both images
        if (img1ok && img2ok) {
          I(x - x0, y - y0) = col1 / byte(2) + col2 / byte(2);
        } else if (img1ok) {
          I(x - x0, y - y0) = col1;
        } else if (img2ok) {
          I(x - x0, y - y0) = col2;
        }
      }
    }
    display(I, 0, 0);
}

// Main function
int main(int argc, char* argv[]) {
    const char* s1 = argc>1? argv[1]: srcPath("image0006.jpg");
    const char* s2 = argc>2? argv[2]: srcPath("image0007.jpg");

    // Load and display images
    Image<Color> I1, I2;
    if( ! load(I1, s1) ||
        ! load(I2, s2) ) {
        cerr<< "Unable to load the images" << endl;
        return 1;
    }
    Window w1 = openWindow(I1.width(), I1.height(), s1);
    display(I1,0,0);
    Window w2 = openWindow(I2.width(), I2.height(), s2);
    setActiveWindow(w2);
    display(I2,0,0);

    // Get user's clicks in images
    vector<IntPoint2> pts1, pts2;
    getClicks(w1, w2, pts1, pts2);

    vector<IntPoint2>::const_iterator it;
    cout << "pts1="<<endl;
    for(it=pts1.begin(); it != pts1.end(); it++)
        cout << *it << endl;
    cout << "pts2="<<endl;
    for(it=pts2.begin(); it != pts2.end(); it++)
        cout << *it << endl;

    // Compute homography
    Matrix<float> H = getHomography(pts1, pts2);
    cout << "H=" << H/H(2,2);

    // Apply homography
    panorama(I1, I2, H);

    endGraphics();
    return 0;
}
