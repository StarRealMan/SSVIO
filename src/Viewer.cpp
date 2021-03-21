#include "Viewer.h"

Viewer::Viewer(XtionCamera::Ptr camera, Odometry::Ptr odometry, Map::Ptr map)
{   
    pangolin::CreateWindowAndBind("RUN_SLAM", 960, 480);
    glEnable(GL_DEPTH_TEST);
    pangolin::GetBoundWindow()->RemoveCurrent();

    _viewer_cam = camera;
    _viewer_odometry = odometry;
    _viewer_map = map;
    
    _viewer_running.store(true);
    _viewer_thread = std::thread(std::bind(&Viewer::ViewerLoop,this));
}

Viewer::~Viewer()
{

}

void Viewer::DrawTrajnTrans(std::vector<Frame::Ptr> key_frames_vec)
{
    for(int i = 0; i < key_frames_vec.size(); i++)
    {
        Eigen::Matrix4f trans = key_frames_vec[i]->GetAbsPose();
        
        glPushMatrix();
        std::vector<GLdouble> Twc = {trans(0, 0), trans(1, 0), trans(2, 0), 0.,
                                     trans(0, 1), trans(1, 1), trans(2, 1), 0.,
                                     trans(0, 2), trans(1, 2), trans(2, 2), 0.,
                                     trans(0, 3), trans(1, 3), trans(2, 3), 1.};
        glMultMatrixd(Twc.data());

        const float w = 0.08;
        const float h = w * 0.75;
        const float z = w * 0.6;

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
    glColor3f(0.f, 1.f, 0.f);
    for(int  i = 0; i < _pose_traj_vec.size() - 1; i++)
    {
        Eigen::Vector3f this_pose = _pose_traj_vec[i];
        Eigen::Vector3f next_pose = _pose_traj_vec[i+1];

        // std::cout << "this pose " << this_pose << std::endl;
        // std::cout << "this pose " << next_pose << std::endl;

        glVertex3d(this_pose.x(), this_pose.y(), this_pose.z());
        glVertex3d(next_pose.x(), next_pose.y(), next_pose.z());
    }
    glEnd();
}


void Viewer::ViewerLoop()
{   
    pangolin::BindToContext("RUN_SLAM");
    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
                                      pangolin::ModelViewLookAt(-2,0,-2, 0,0,0, pangolin::AxisY));
    
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

    while(_viewer_running.load())
    {
        auto t1 = std::chrono::steady_clock::now();
     
        _cur_frame = _viewer_odometry->GetCurFrame();

        if(!(_cur_frame->GetRGBImage().empty()))
        {
            Frame::Ptr cur_frame = _viewer_odometry->GetCurFrame();
            _pose_traj_vec.push_back(cur_frame->GetAbsPose().block<3,1>(0,3));

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);

            DrawTrajnTrans(_viewer_map->GetKeyFramesVec());

            cv::Mat depth_img = _cur_frame->GetDImage();
            depth_img.convertTo(depth_img, CV_8U, 255.0/10000);
            cv::cvtColor(depth_img, depth_img, cv::COLOR_GRAY2BGR);

            cv::Mat rgb_img = _cur_frame->GetRGBImage();
            cv::drawKeypoints(rgb_img, _cur_frame->GetKeyPoints(), rgb_img);
            
            imgTexture1.Upload(depth_img.data, GL_BGR, GL_UNSIGNED_BYTE);
            imgTexture2.Upload(rgb_img.data, GL_BGR, GL_UNSIGNED_BYTE);

            cv_img_1.Activate();
            glColor3f(1.0f, 1.0f, 1.0f);
            imgTexture1.RenderToViewportFlipY();
            
            cv_img_2.Activate();
            glColor3f(1.0f, 1.0f, 1.0f);
            imgTexture2.RenderToViewportFlipY();
            
            pangolin::FinishFrame();

            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // std::cout << "Viewer Thread " << time_used.count()*1000 << " ms per frame " << std::endl;
    }
}

void Viewer::ViewerStop()
{
    _viewer_running.store(false);
    _viewer_thread.join();
}
