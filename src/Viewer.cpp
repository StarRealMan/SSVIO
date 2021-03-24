#include "Viewer.h"

Viewer::Viewer(XtionCamera::Ptr camera, Odometry::Ptr odometry, Map::Ptr map):_pcl_viewer("Simple Cloud Viewer")
{   
    _viewer_cam = camera;
    _viewer_odometry = odometry;
    _viewer_map = map;
}

Viewer::~Viewer()
{

}


void Viewer::DrawTrajnTrans()
{
    std::vector<Frame::Ptr>& key_frames_vec = _viewer_map->GetKeyFramesVec();
    std::vector<Eigen::Vector3f>& pose_traj_vec = _viewer_map->GetTrajVec();


    glLineWidth(2); 
    glBegin(GL_LINES);
    glColor3f(1.f,0.f,0.f);
    glVertex3f(0,0,0);		glVertex3f(1.f,0,0);
    glColor3f(0.f,0.f,1.f);
    glVertex3f(0,0,0);		glVertex3f(0,1.f,0);
    glColor3f(0.f,1.f,0.f);
    glVertex3f(0,0,0);		glVertex3f(0,0,1.f);
    glEnd();

    for(size_t i = 0; i < key_frames_vec.size(); i++)
    {
        Eigen::Matrix4f trans = key_frames_vec[i]->GetAbsPose().inverse();

        glPushMatrix();
        std::vector<GLdouble> Twc = {trans(0, 0), trans(1, 0), trans(2, 0), 0.,
                                     trans(0, 1), trans(1, 1), trans(2, 1), 0.,
                                     trans(0, 2), trans(1, 2), trans(2, 2), 0.,
                                     trans(0, 3), trans(1, 3), trans(2, 3), 1.};
        glMultMatrixd(Twc.data());

        float w = 0.02;
        if(i == key_frames_vec.size() - 1)
        {
            w = 0.12;
        }
        float h = w * 0.75;
        float z = w * 0.6;

        glLineWidth(1); 
        glBegin(GL_LINES);
        glColor3f(1.0f,1.0f,0.0f);
        glVertex3f(0,0,0);		glVertex3f(w,h,z);
        glVertex3f(0,0,0);		glVertex3f(w,-h,z);
        glVertex3f(0,0,0);		glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);		glVertex3f(-w,h,z);
        glVertex3f(w,h,z);		glVertex3f(w,-h,z);
        glVertex3f(-w,h,z);		glVertex3f(-w,-h,z);
        glVertex3f(-w,h,z);		glVertex3f(w,h,z);
        glVertex3f(-w,-h,z);    glVertex3f(w,-h,z);
        glEnd();
        glPopMatrix();
    }

    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3f(0.f, 1.f, 1.f);
    for(size_t  i = 0; i < pose_traj_vec.size() - 1; i++)
    {
        Eigen::Vector3f this_pose = pose_traj_vec[i];
        Eigen::Vector3f next_pose = pose_traj_vec[i+1];

        glVertex3d(this_pose.x(), this_pose.y(), this_pose.z());
        glVertex3d(next_pose.x(), next_pose.y(), next_pose.z());
    }
    glEnd();
}

void Viewer::ViewerLoop()
{   
    cv::Mat rgb_img;
    cv::Mat depth_img;

    pangolin::CreateWindowAndBind("RUN_SLAM", 960, 480);
    glEnable(GL_DEPTH_TEST);
    
    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
                                      pangolin::ModelViewLookAt(0,0,-0.5, 0,0,0, pangolin::AxisNegY));
    
    pangolin::View& d_cam = pangolin::Display("cam")
        .SetBounds(0., 1., 1/3.0f, 1., -640/480.)
        .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::View& cv_img_1 = pangolin::Display("RGBImg")
        .SetBounds(1/2.0f, 1.0f, 0., 1/3.0f, 320/240.)
        .SetLock(pangolin::LockLeft, pangolin::LockTop);
    pangolin::View& cv_img_2 = pangolin::Display("DepthImg")
        .SetBounds(0., 1/2.0f, 0.,1/3.0f, 320/240.)
        .SetLock(pangolin::LockLeft, pangolin::LockBottom);

    pangolin::GlTexture imgTexture1(320, 240, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
    pangolin::GlTexture imgTexture2(320, 240, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);


    while(!pangolin::ShouldQuit())
    {
        auto t1 = std::chrono::steady_clock::now();
     
        _cur_frame = _viewer_odometry->GetCurFrame();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        
        DrawTrajnTrans();

        if(!(_cur_frame->GetRGBImage().empty()))
        {
            _cur_frame->GetDImage().convertTo(depth_img, CV_8U, 255.0/65535);
            cv::cvtColor(depth_img, depth_img, cv::COLOR_GRAY2BGR);
            cv::drawKeypoints(_cur_frame->GetRGBImage(), _cur_frame->GetKeyPoints(), rgb_img);
            
            imgTexture1.Upload(depth_img.data, GL_BGR, GL_UNSIGNED_BYTE);
            imgTexture2.Upload(rgb_img.data, GL_BGR, GL_UNSIGNED_BYTE);

            cv_img_1.Activate();
            glColor3f(1.0f, 1.0f, 1.0f);
            imgTexture1.RenderToViewportFlipY();
            cv_img_2.Activate();
            glColor3f(1.0f, 1.0f, 1.0f);
            imgTexture2.RenderToViewportFlipY();
        }
          
        pangolin::FinishFrame();

        _pcl_viewer.showCloud(_viewer_cam->GetRGBCloud());

        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // std::cout << "Viewer Thread " << time_used.count()*1000 << " ms per frame " << std::endl;
    }
}
