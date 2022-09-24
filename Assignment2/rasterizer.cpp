// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, _v);
    if (alpha >= 0 && beta >= 0 && gamma >= 0) return true;
    else return false;
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}




//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    auto A = v[0];
    auto B = v[1];
    auto C = v[2];
    int x_min = floor(std::min({A.x(), B.x(), C.x()}));
    int x_max = ceil(std::max({A.x(), B.x(), C.x()}));
    int y_min = floor(std::min({A.y(), B.y(), C.y()}));
    int y_max = ceil(std::max({A.y(), B.y(), C.y()}));

    // iterate through the pixel and find if the current pixel is inside the triangle
    AA_type current = MSAA;
    if(current == Vanilla)
    {
        for (int i = x_min; i < x_max; i++)
        {
            for ( int j = y_min; j < y_max; j++)
            {
                if (insideTriangle(i,j,t.v))
                {
                    auto[alpha, beta, gamma] = computeBarycentric2D(i+0.5, j+0.5, t.v);
                    //Compute interpolated z value for depth buffer
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    // check z value, if the current value is smaller than what is stored, fill in new value 
                    if (z_interpolated < depth_buf[get_index(i, j)])
                    {
                        depth_buf[get_index(i, j)] = z_interpolated;
                        Eigen::Vector3f p(i, j, 0);
                        set_pixel(p, t.getColor());
                    }
                }
            }
        }
    }
    else if(current == SSAA)
    {
        // Fill a larger framebuffer and z-buffer
        for (int i = x_min; i < x_max; i++)
        {
            for ( int j = y_min; j < y_max; j++)
            {
                // iterate over this pixel
                for ( auto x = 0; x < SSAA_width; x++)
                {
                    for ( auto y = 0; y < SSAA_height; y++)
                    {
                        if (insideTriangle(i + SSAA_bin_width * x, j + SSAA_bin_height * y, t.v))
                        {
                            auto[alpha, beta, gamma] = computeBarycentric2D(i + SSAA_bin_width * x, j + SSAA_bin_height * y, t.v);
                            //Compute interpolated z value for depth buffer
                            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;
                            // check z value, if the current value is smaller than what is stored, fill in new value 
                            if (z_interpolated < depth_buf_SSAA[get_index_SSAA(i,j,x,y)])
                            {
                                depth_buf_SSAA[get_index_SSAA(i,j,x,y)] = z_interpolated;
                                frame_buf_SSAA[get_index_SSAA(i,j,x,y)] = t.getColor();
                            }
                        }
                    }
                }

            }
        }
        // downsampling

        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                Eigen::Vector3f total_color = {0., 0., 0.};
                for(int i = 0; i < SSAA_width; i++) {
                    for(int j = 0; j < SSAA_height; j++) {
                        int index = get_index_SSAA(x, y, i, j);
                        total_color += frame_buf_SSAA[index];
                    }
                }
                Eigen::Vector3f p(x, y, 0);
                set_pixel(p, total_color / (SSAA_width*SSAA_height));
            }
        }
    }
    else if (current == MSAA)
    {
        for (int i = x_min; i < x_max; i++)
        {
            for ( int j = y_min; j < y_max; j++)
            {
                // MSAA count
                float count = 0;
                for (int x = 0; x < MSAA_width; x++)
                {
                    for (int y = 0; y < MSAA_height; y++)
                    {
                        if ( insideTriangle(i + x*MSAA_bin_width, j + y*MSAA_bin_height, t.v))
                        {
                            count += 1.0;
                        }
                    }
                } 

                if (insideTriangle(i,j,t.v))
                {
                    auto[alpha, beta, gamma] = computeBarycentric2D(i+0.5, j+0.5, t.v);
                    //Compute interpolated z value for depth buffer
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    // check z value, if the current value is smaller than what is stored, fill in new value 
                    if (z_interpolated < depth_buf[get_index(i, j)])
                    {
                        depth_buf[get_index(i, j)] = z_interpolated;
                        Eigen::Vector3f p(i, j, 0);
                        set_pixel(p, t.getColor()*count/(MSAA_width*MSAA_height));
                    }
                }
                // To remove black border, but only works if only two triangle intersects
                // Rigorous way should keep a width*height*MSAA_height*MSAA_width depth buffer
                if(count < MSAA_height*MSAA_width) {
                    depth_buf[get_index(i, j)] = std::numeric_limits<float>::infinity();
                }
            }
        }
    }


    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf_SSAA.begin(), frame_buf_SSAA.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_SSAA.begin(), depth_buf_SSAA.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_SSAA.resize(w * SSAA_width * h * SSAA_height);
    depth_buf_SSAA.resize(w * SSAA_width * h * SSAA_height);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_index_SSAA(int x, int y, int i, int j)
{
    // given point with original frame buffer pixel index (x,y), 
    // and the (i,j) index inside this pixel
    // the function returns the calculate index in the (i_,j_) in the SSAA frame buffer image
    // finally return the index of SSAA framebuffer as a linear 1D array
    int total_height = height * SSAA_height;
    int total_width = width * SSAA_width;

    int i_ = x * SSAA_width + i;
    int j_ = y * SSAA_height + j;

    // (0,width) is the bottom right corner
    // (height, width) is the top left corner
    return (total_height-1-j_)*total_width + i_;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on