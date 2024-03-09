// A practical implementation of the rasterization algorithm in software.

#include "geometry.h"
#include <SDL.h>
#include "model.h"
#include <fstream>
#include <chrono>

#define M_PI 3.14159265359

static const float inchToMm = 25.4;
enum FitResolutionGate { kFill = 0, kOverscan };

// Compute screen coordinates based on a physically-based camera model
// http://www.scratchapixel.com/lessons/3d-basic-rendering/3d-viewing-pinhole-camera
void computeScreenCoordinates(
    const float &filmApertureWidth,
    const float &filmApertureHeight,
    const uint32_t &imageWidth,
    const uint32_t &imageHeight,
    const FitResolutionGate &fitFilm,
    const float &nearClippingPlane,
    const float &focalLength,
    float &top, float &bottom, float &left, float &right
)
{
    float filmAspectRatio = filmApertureWidth / filmApertureHeight;
    float deviceAspectRatio = imageWidth / (float)imageHeight;
    
    top = ((filmApertureHeight * inchToMm / 2) / focalLength) * nearClippingPlane;
    right = ((filmApertureWidth * inchToMm / 2) / focalLength) * nearClippingPlane;

    // field of view (horizontal)
    float fov = 2 * 180 / M_PI * atan((filmApertureWidth * inchToMm / 2) / focalLength);
    std::cerr << "Field of view " << fov << std::endl;
    
    float xscale = 1;
    float yscale = 1;
    
    switch (fitFilm) {
        default:
        case kFill:
            if (filmAspectRatio > deviceAspectRatio) {
                xscale = deviceAspectRatio / filmAspectRatio;
            }
            else {
                yscale = filmAspectRatio / deviceAspectRatio;
            }
            break;
        case kOverscan:
            if (filmAspectRatio > deviceAspectRatio) {
                yscale = filmAspectRatio / deviceAspectRatio;
            }
            else {
                xscale = deviceAspectRatio / filmAspectRatio;
            }
            break;
    }
    
    right *= xscale;
    top *= yscale;
    
    bottom = -top;
    left = -right;
}

// Compute vertex raster screen coordinates.
// Vertices are defined in world space. They are then converted to camera space,
// then to NDC space (in the range [-1,1]) and then to raster space.
// The z-coordinates of the vertex in raster space is set with the z-coordinate
// of the vertex in camera space.
void convertToRaster(
    const Vec3f &vertexWorld,
    const Matrix44f &worldToCamera,
    const float &l,
    const float &r,
    const float &t,
    const float &b,
    const float &near,
    const uint32_t &imageWidth,
    const uint32_t &imageHeight,
    Vec3f &vertexRaster
)
{
    // TASK 1
    // convert to camera space - your implementation here
    // define a new Vec3f to store the camera space position of the vertex (vertexCamera)
    // multiple the worldToCamera matrix with the vertexWorld position
    // store the value in vertexCamera
    Vec3f vertexCamera;
    worldToCamera.multVecMatrix(vertexWorld, vertexCamera);

    // TASK 2
    // convert to screen space - your implementation here
    // define a Vec2f to store the vertex screen space position (vertexScreen)
    // calculate x and y components for this position from the vertexCamera variable
    Vec2f vertexScreen;
    vertexScreen.x = near * vertexCamera.x / -vertexCamera.z;
    vertexScreen.y = near * vertexCamera.y / -vertexCamera.z;
    float screenDepth = -vertexCamera.z;
    
    // TASK 3
    // now convert point from screen space to NDC space (in range [-1,1])
    // - your implementation here
    // define a new Vec2f to support the NDC position (vertexNDC)
    // calculate this position and store in vertexNDC
    Vec2f vertexNDC;
    vertexNDC.x = 2 * vertexScreen.x / (r - l) - (r + l) / (r - l);
    vertexNDC.y = 2 * vertexScreen.y / (t - b) - (t + b) / (t - b);

    // TASK 4
    // convert to raster space  - your implementation here
    // vertexRaster is passed as reference and is where you store the output of this calculation
    // convert from NDCs to raster position based upon image width and height
    // store the z component in vertexRaster as the -ve depth from vertexCamera - used later in depth testing
    // -ve depth actually makes this a positive depth value (as camera looks down -ve z)
    // in raster space y is down so invert direction
    vertexRaster.x = (vertexNDC.x + 1) * 0.5 * imageWidth;
    vertexRaster.y = (1 - vertexNDC.y) * 0.5 * imageHeight;
    vertexRaster.z = screenDepth;
}

float min3(const float &a, const float &b, const float &c)
{ return std::min(a, std::min(b, c)); }

float max3(const float &a, const float &b, const float &c)
{ return std::max(a, std::max(b, c)); }

float edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{ return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]); }

const uint32_t imageWidth = 640;
const uint32_t imageHeight = 480;
Matrix44f worldToCamera;

const float nearClippingPlane = 1;
const float farClippingPlane = 1000;
float focalLength = 20; // in mm
// 35mm Full Aperture in inches
float filmApertureWidth = 0.980;
float filmApertureHeight = 0.735;

SDL_Window* window;
SDL_Renderer* renderer;
SDL_Surface* screen;
void init() {
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window* window = SDL_CreateWindow(
        "Software Rasteriser",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        640,
        480,
        0
    );

    screen = SDL_GetWindowSurface(window);

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
}

void putpixel(SDL_Surface* surface, int x, int y, Uint32 pixel)
{
    int bpp = surface->format->BytesPerPixel;
    /* Here p is the address to the pixel we want to set */
    Uint8* p = (Uint8*)surface->pixels + y * surface->pitch + x * bpp;

    switch (bpp) {
    case 1:
        *p = pixel;
        break;

    case 2:
        *(Uint16*)p = pixel;
        break;

    case 3:
        if (SDL_BYTEORDER == SDL_BIG_ENDIAN) {
            p[0] = (pixel >> 16) & 0xff;
            p[1] = (pixel >> 8) & 0xff;
            p[2] = pixel & 0xff;
        }
        else {
            p[0] = pixel & 0xff;
            p[1] = (pixel >> 8) & 0xff;
            p[2] = (pixel >> 16) & 0xff;
        }
        break;

    case 4:
        *(Uint32*)p = pixel;
        break;
    }
}

Matrix44f lookAt(const Vec3f from, const Vec3f to, const Vec3f _tmp = Vec3f(0, 1, 0))
{
    // TASK 5
    // Calculate forward, right and up vectors
    // Hint: these should be at right angles to one another - when you want to find 
    // a vector perpendicular to another, think of the cross product!
    // Extra Hint: try to do this yourself first, but if you're stuck on this task,
    // check the LookAt implementation in the Lecture 6: Viewing slides.

    Matrix44f camToWorld;
    Vec3f forward = (from - to);
    forward.normalize();
    Vec3f right = _tmp.crossProduct(forward);
    right.normalize();
    Vec3f up = forward.crossProduct(right);
    up.normalize();

    // Create a 4x4 orientation matrix from the right, up, and forward vectors
    Matrix44f orientation = {
       right.x, right.y, right.z, 0,
       up.x, up.y, up.z, 0,
       forward.x, forward.y, forward.z, 0,
       0,0,0,1
    };

    // Create a 4x4 translation matrix.
    // The eye position is negated which is equivalent
    // to the inverse of the translation matrix. 
    Matrix44f translation = {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        from.x,from.y,from.z,1
    };

    // Combine the orientation and translation to compute 
    // the final view matrix. Note that the order of 
    // multiplication is reversed because the matrices
    // are already inverted.
    //return (translation * orientation);
    return (orientation * translation);
    //return (translation * orientation);
}

Model* model = nullptr;

int main(int argc, char **argv)
{
    // load model
    if (2 == argc) {
        model = new Model(argv[1]);
    }
    else {
        model = new Model("../cc_t.obj");
    }

    // initialise SDL2
    init();

    // compute screen coordinates
    float t, b, l, r;
    
    // Calculate screen coordinates and store in t, b, l, r
    computeScreenCoordinates(
        filmApertureWidth, filmApertureHeight,
        imageWidth, imageHeight,
        kOverscan,
        nearClippingPlane,
        focalLength,
        t, b, l, r);
    
    // define the depth-buffer. Initialize depth buffer to far clipping plane.
    float *depthBuffer = new float[imageWidth * imageHeight];
    for (uint32_t i = 0; i < imageWidth * imageHeight; ++i) depthBuffer[i] = farClippingPlane;

    float angle = 0.0f;

    float dist = 3.0f;

    SDL_Event e;
    bool running = true;
    while (running) {

        // Start timer so we can gather frame generation statistics
        auto t_start = std::chrono::high_resolution_clock::now();

        // clear back buffer, pixel data on surface and depth buffer (as movement)
        SDL_FillRect(screen, nullptr, SDL_MapRGB(screen->format, 0, 0, 0));
        SDL_RenderClear(renderer);
        // Only required if animating the camera as the depth buffer will need to be recomputed
        for (uint32_t i = 0; i < imageWidth * imageHeight; ++i) depthBuffer[i] = farClippingPlane;

        // Hard coded worldToCamera matrix
        worldToCamera = { 1,0,0,0,
                          0,1,0.3,0,
                          0,-0.3,0.9,0,
                          0,0,-3,1 };

        // TASK 5
        // Currently the worldToCamera matrix is hard coded, and if convertToRaster is correctly implemented you will 
        // have an image of the model rasterised and displayed to the SDL_Window. 
        // You now need to create a system for a camera matrix to be constructed. The easiest way of doing this is by the 
        // lookAt() method described in the lecture notes for the Viewing Transformation series. 
        // A stub of this method is within this code and guidance on implementing it is here:
        // https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/lookat-function
        // For eye, target and up Vec3's you should be able to return a cameraToWorld matrix to invert for a worldToCamera matrix:
        Vec3f eye(4.f, 0.f, 0.f);
        Vec3f target(0.f, 0.0f, 0.f);
        Vec3f up(0.f, 1.f, 0.f);
        // worldToCamera = lookAt(eye, target, up).inverse();
        // now implement the lookAt() method!
        //worldToCamera = lookAt(eye, target, up).inverse();

        // TASK 6 
        // Implement the Arcball Camera to replace Vec3f eye(0.f, 1.f, 3.f); with Vec3f eye(camX, camY, camZ); computed each frame
        // for increments of camAngleX, starting at 0.0f and resetting after incrementing past 360 degrees. 
        angle += 0.101f;
        while (angle > 2.0 * M_PI) angle -= 2.0f * M_PI;
        eye = Vec3f(dist * sinf(angle), 0.f, dist * cosf(angle));
        worldToCamera = lookAt(eye, target, up).inverse();

        // Outer loop - For every face in the model (would eventually need to be amended to be for every face in every model)
        for (uint32_t i = 0; i < model->nfaces(); ++i) {
            // v0, v1 and v2 store the vertex positions of every vertex of the 3D model
            const Vec3f& v0 = model->vert(model->face(i)[0]);// + Vec3f(0, 0, -3);
            const Vec3f& v1 = model->vert(model->face(i)[1]);// +Vec3f(0, 0, -3);
            const Vec3f& v2 = model->vert(model->face(i)[2]);// + Vec3f(0, 0, -3);

            // Convert the vertices of the triangle to raster space - you will need to implement convertToRaster()
            Vec3f v0Raster, v1Raster, v2Raster;
            convertToRaster(v0, worldToCamera, l, r, t, b, nearClippingPlane, imageWidth, imageHeight, v0Raster);
            convertToRaster(v1, worldToCamera, l, r, t, b, nearClippingPlane, imageWidth, imageHeight, v1Raster);
            convertToRaster(v2, worldToCamera, l, r, t, b, nearClippingPlane, imageWidth, imageHeight, v2Raster);

            // Precompute reciprocal of vertex z-coordinate
            v0Raster.z = 1 / v0Raster.z,
            v1Raster.z = 1 / v1Raster.z,
            v2Raster.z = 1 / v2Raster.z;

            // Prepare vertex attributes. Divide them by their vertex z-coordinate
            // (though we use a multiplication here because v.z = 1 / v.z)
            // st0, st1 and st2 store the texture coordinates from the model of each vertex
            Vec2f st0 = model->vt(model->face(i)[0]);
            Vec2f st1 = model->vt(model->face(i)[1]);
            Vec2f st2 = model->vt(model->face(i)[2]);
            st0 *= v0Raster.z, st1 *= v1Raster.z, st2 *= v2Raster.z;

            // Calculate the bounding box of the triangle defined by the vertices
            float xmin = min3(v0Raster.x, v1Raster.x, v2Raster.x);
            float ymin = min3(v0Raster.y, v1Raster.y, v2Raster.y);
            float xmax = max3(v0Raster.x, v1Raster.x, v2Raster.x);
            float ymax = max3(v0Raster.y, v1Raster.y, v2Raster.y);

            // the triangle is out of screen
            if (xmin > imageWidth - 1 || xmax < 0 || ymin > imageHeight - 1 || ymax < 0) continue;

            // avoid drawing huge triangles
            if (xmax - xmin > 200 || ymax - ymin > 200) continue;

            // sets the bounds of the rectangle for the raster triangle
            // be careful xmin/xmax/ymin/ymax can be negative. Don't cast to uint32_t
            uint32_t x0 = std::max(int32_t(0), (int32_t)(std::floor(xmin)));
            uint32_t x1 = std::min(int32_t(imageWidth) - 1, (int32_t)(std::floor(xmax)));
            uint32_t y0 = std::max(int32_t(0), (int32_t)(std::floor(ymin)));
            uint32_t y1 = std::min(int32_t(imageHeight) - 1, (int32_t)(std::floor(ymax)));
            // calculates the area of the triangle, used in determining barycentric coordinates
            float area = edgeFunction(v0Raster, v1Raster, v2Raster);

            // Inner loop - for every pixel of the bounding box enclosing the triangle
            for (uint32_t y = y0; y <= y1; ++y) {
                for (uint32_t x = x0; x <= x1; ++x) {
                    Vec3f pixelSample(x + 0.5, y + 0.5, 0); // You could use multiple pixel samples for antialiasing!!
                    // Calculate the area of the subtriangles for barycentric coordinates
                    float w0 = edgeFunction(v1Raster, v2Raster, pixelSample);
                    float w1 = edgeFunction(v2Raster, v0Raster, pixelSample);
                    float w2 = edgeFunction(v0Raster, v1Raster, pixelSample);
                    if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                        // divide by the area to give us our coefficients
                        w0 /= area;
                        w1 /= area;
                        w2 /= area;
                        float oneOverZ = v0Raster.z * w0 + v1Raster.z * w1 + v2Raster.z * w2; // reciprocal for depth testing
                        float z = 1 / oneOverZ;
                        // Depth-buffer test
                        if (z < depthBuffer[y * imageWidth + x]) { // is this triangle closer than others previously?
                            depthBuffer[y * imageWidth + x] = z;

                            // Calculate the texture coordinate based on barycentric position of the pixel
                            Vec2f st = st0 * w0 + st1 * w1 + st2 * w2;

                            // correct for perspective distortion
                            st *= z;

                            // If you need to compute the actual position of the shaded
                            // point in camera space. Proceed like with the other vertex attribute.
                            // Divide the point coordinates by the vertex z-coordinate then
                            // interpolate using barycentric coordinates and finally multiply
                            // by sample depth.
                            Vec3f v0Cam, v1Cam, v2Cam;
                            worldToCamera.multVecMatrix(v0, v0Cam);
                            worldToCamera.multVecMatrix(v1, v1Cam);
                            worldToCamera.multVecMatrix(v2, v2Cam);

                            float px = (v0Cam.x / -v0Cam.z) * w0 + (v1Cam.x / -v1Cam.z) * w1 + (v2Cam.x / -v2Cam.z) * w2;
                            float py = (v0Cam.y / -v0Cam.z) * w0 + (v1Cam.y / -v1Cam.z) * w1 + (v2Cam.y / -v2Cam.z) * w2;

                            Vec3f pt(px * z, py * z, -z); // pt is in camera space

                            // Compute the face normal which is used for a simple facing ratio.
                            // Keep in mind that we are doing all calculation in camera space.
                            // Thus the view direction can be computed as the point on the object
                            // in camera space minus Vec3f(0), the position of the camera in camera space.
                            Vec3f n = (v1Cam - v0Cam).crossProduct(v2Cam - v0Cam);
                            n.normalize();
                            Vec3f viewDirection = -pt;
                            viewDirection.normalize();

                            // Calculate shading of the surface based on dot product of the normal and view direction
                            float nDotView = std::max(0.f, n.dotProduct(viewDirection));

                            // The final color is the result of the fraction multiplied by the
                            // checkerboard pattern defined in checker.
                            const int M = 10;
                            float checker = (fmod(st.x * M, 1.0) > 0.5) ^ (fmod(st.y * M, 1.0) < 0.5);
                            float c = 0.3 * (1 - checker) + 0.7 * checker;
                            nDotView *= c;

                            // Set the pixel value on the SDL_Surface that gets drawn to the SDL_Window
                            Uint32 colour = SDL_MapRGB(screen->format, nDotView * 255, nDotView * 255, nDotView * 255);
                            putpixel(screen, x, y, colour);
                        }
                    }
                }
            }
        }

        // Calculate frame interval timing
        auto t_end = std::chrono::high_resolution_clock::now();
        auto passedTime = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        std::cerr << "Frame render time:  " << passedTime << " ms" << std::endl;

        // Create texture from the surface and RenderCopy/Present from backbuffer
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, screen);
        if (texture == NULL) {
            fprintf(stderr, "CreateTextureFromSurface failed: %s\n", SDL_GetError());
            exit(1);
        }
        SDL_FreeSurface(screen);

        SDL_RenderCopy(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        // Clean up heap allocation
        SDL_DestroyTexture(texture);

        // Check for ESC sequence, otherwise keep drawing frames
        if (SDL_PollEvent(&e))
        {
            switch (e.type) {
            case SDL_KEYDOWN:
                switch (e.key.keysym.sym) {
                case SDLK_ESCAPE:
                    running = false;
                    break;
                }
                break;
            }
        }
    }

    // tidy up dangling pointer to the depth buffer
    delete [] depthBuffer;

    return 0;
}
