#include "HandEyeCali3D.h"

#include <QFileDialog>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

#include <vtkAutoInit.h>

#include <unsupported/Eigen/KroneckerProduct>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

THandEyeCali3D::THandEyeCali3D(TCameraChishine* a_camera, QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	this->setFixedSize(this->width(), this->height());

	m_camera_chishine_handeye3D = a_camera;

	std::cout << "进入3D标定模式..." << std::endl << std::endl;

	init_vtkwidget();
	m_handeye_cali_data.clear();//清空标定数据
	m_mat_X = Eigen::Matrix4d::Zero();//手眼矩阵置零
	m_mat_regis = Eigen::Matrix4f::Zero();//配准矩阵置零

	if (m_camera_chishine_handeye3D->open_camera())
	{
		//更改ui状态
		ui.btn_load_CAD->setEnabled(true);
		ui.btn_get_pointcloud->setEnabled(true);
		ui.btn_roi_cut->setEnabled(true);
		ui.btn_registration->setEnabled(true);
		ui.btn_save_cali_data->setEnabled(true);
		ui.btn_cal_hadeye_mat->setEnabled(true);
	}
}

THandEyeCali3D::~THandEyeCali3D()
{
	ui.vtk_cad->GetInteractor()->SetRenderWindow(nullptr);
	ui.vtk_cad->GetInteractor()->SetInteractorStyle(nullptr);
	ui.vtk_cam->GetInteractor()->SetRenderWindow(nullptr);
	ui.vtk_cam->GetInteractor()->SetInteractorStyle(nullptr);
	ui.vtk_ret->GetInteractor()->SetRenderWindow(nullptr);
	ui.vtk_ret->GetInteractor()->SetInteractorStyle(nullptr);
}

void THandEyeCali3D::on_btn_load_CAD_clicked()
{
	QString fileName = QFileDialog::getOpenFileName(this, "Open PointCloud", "", "ply(*.ply);;pcd(*.pcd)");
	if (fileName.isEmpty())
		return;
	std::string filepath = fileName.toStdString();
	std::string ext = filepath.substr(filepath.size() - 3, 3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (ext == "ply")
	{
		if (-1 == pcl::io::loadPLYFile(filepath, *cloud)) //打开点云文件
		{
			std::cout << "打开ply点云文件失败" << std::endl;
			return;
		}
	}

	if (ext == "pcd")
	{
		if (-1 == pcl::io::loadPCDFile(filepath, *cloud)) //打开点云文件
		{
			std::cout << "打开pcd点云文件失败" << std::endl;
			return;
		}
	}

	m_viewer_cad->removeAllPointClouds();//清空viewer内的点云

	m_cloud_cad->clear();
	for (auto it = cloud->cbegin(); it != cloud->end(); it++)
	{
		if (it->x == 0 && it->y == 0 && it->z == 0)
			continue;
		m_cloud_cad->emplace_back(it->x, it->y, it->z);
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(m_cloud_cad, 0, 255, 0); //CAD点云颜色为绿色
	m_viewer_cad->addPointCloud(m_cloud_cad, src_color);//viewer添加点云
	ui.vtk_cad->update();//vtk更新

	std::cout << "CAD点云加载成功" << std::endl;
}

void THandEyeCali3D::on_btn_get_pointcloud_clicked()
{
	pcl::PointCloud<pcl::PointNormal> cloud;
	if (!m_camera_chishine_handeye3D->get_pointcloud_api(cloud))
	{
		std::cout << "点云获取失败" << std::endl;
		return;
	}
	else
	{
		m_viewer_cam->removeAllPointClouds();//清空viewer内的点云
		m_cloud_cam->clear();

		for (auto it = cloud.cbegin(); it != cloud.cend(); it++)
		{
			if (it->x == 0 && it->y == 0 && it->z == 0)
				continue;
			m_cloud_cam->emplace_back(it->x, it->y, it->z);
		}

		//vtk显示

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_color(m_cloud_cam, 0, 0, 255); //CAD点云颜色为绿色
		m_viewer_cam->addPointCloud(m_cloud_cam, src_color);//viewer添加点云
		ui.vtk_cam->update();//vtk更新

		std::cout << "相机点云获取成功" << std::endl;
	}
}

void THandEyeCali3D::on_btn_roi_cut_clicked()
{

}

void THandEyeCali3D::on_btn_registration_clicked()
{
	m_mat_regis.setZero();

	if (m_cloud_cad->size() == 0)
	{
		std::cout << "CAD点云为空!" << std::endl;
		return;
	}
	if (m_cloud_cam->size() == 0)
	{
		std::cout << "相机点云为空!" << std::endl;
		return;
	}

	m_mat_regis = TRegisAlgorithm::icp_point_point(m_cloud_cam, m_cloud_cad);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*m_cloud_cam, *cloud_trans, m_mat_regis);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cad(m_cloud_cad);

	m_viewer_ret->removeAllPointClouds();//清空viewer内的点云
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cad_color(cloud_cad, 0, 255, 0); //CAD点云颜色为绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> trans_color(cloud_trans, 0, 0, 255); //相机点云颜色为蓝色
	m_viewer_ret->addPointCloud(cloud_cad, cad_color, "cloud cad");
	m_viewer_ret->addPointCloud(cloud_trans, trans_color, "cloud cam");
	ui.vtk_ret->update();//vtk更新
}

void THandEyeCali3D::on_btn_save_cali_data_clicked()
{
	TPoint6D position;
	if (!m_camera_chishine_handeye3D->get_TPoint6D(position))
	{
		std::cout << "获取当前TCP失败，请重新保存标定数据" << std::endl;
		return;
	}
	//if (!m_cloud_cad)
	//{
	//	std::cout << "CAD点云为空，无标定数据" << std::endl;
	//	return;
	//}
	//if (!m_cloud_cam)
	//{
	//	std::cout << "相机点云为空，无标定数据" << std::endl;
	//	return;
	//}
	if (m_mat_regis.isZero())
	{
		std::cout << "配准矩阵为空,请重新配准" << std::endl;
		return;
	}

	THandEyeCaliData data;
	data.TCP_position = position;
	for (int i = 0; i < m_mat_regis.rows(); i++)
	{
		for (int j = 0; j < m_mat_regis.cols(); j++)
		{
			data.mat_cam2CAD(i, j) = (double)m_mat_regis(i, j);
		}
	}
	m_handeye_cali_data.push_back(data);
	std::cout << "保存标定数据成功" << std::endl;
}

void THandEyeCali3D::on_btn_cal_hadeye_mat_clicked()
{
	if (m_handeye_cali_data.size() < 5)
	{
		std::cout << "手眼标定数据不足，请继续添加标定数据" << std::endl;
		return;
	}

	//计算A矩阵和B矩阵
	Poses A_;
	Poses B_;
	Eigen::Matrix4d handeye_mat;
	for (int i = 1; i < m_handeye_cali_data.size(); i++)
	{
		A_.push_back(tcp2tcp(m_handeye_cali_data[0].TCP_position, m_handeye_cali_data[i].TCP_position));
		B_.push_back(mat2mat(m_handeye_cali_data[0].mat_cam2CAD, m_handeye_cali_data[i].mat_cam2CAD));
	}

	if (solveAXXB_1(A_, B_, handeye_mat))
	{
		std::cout << "手眼矩阵计算成功" << std::endl;
		m_mat_X = handeye_mat;
		std::cout << m_mat_X << std::endl;
	}
	else
	{
		std::cout << "手眼矩阵计算失败" << std::endl;
	}
}

void THandEyeCali3D::init_vtkwidget()
{
	m_cloud_cad.reset(new pcl::PointCloud<pcl::PointXYZ>);
	m_cloud_cam.reset(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_init(new pcl::PointCloud<pcl::PointXYZ>);
	m_viewer_cad.reset(new pcl::visualization::PCLVisualizer("viewer CAD", false));
	m_viewer_cam.reset(new pcl::visualization::PCLVisualizer("viewer camera", false));
	m_viewer_ret.reset(new pcl::visualization::PCLVisualizer("viewer registration result", false));
	m_viewer_cad->addPointCloud(m_cloud_cad, "cloud of CAD");
	m_viewer_cam->addPointCloud(m_cloud_cam, "cloud of camera");
	//m_viewer_ret->addPointCloud(tmp_init, "cloud of initial");
	m_viewer_cad->setBackgroundColor(255, 255, 255);
	m_viewer_cam->setBackgroundColor(255, 255, 255);
	m_viewer_ret->setBackgroundColor(255, 255, 255);
	m_viewer_cad->addCoordinateSystem(1.0);
	m_viewer_cam->addCoordinateSystem(1.0);
	m_viewer_ret->addCoordinateSystem(1.0);
	ui.vtk_cad->SetRenderWindow(m_viewer_cad->getRenderWindow());
	ui.vtk_cam->SetRenderWindow(m_viewer_cam->getRenderWindow());
	ui.vtk_ret->SetRenderWindow(m_viewer_ret->getRenderWindow());
	m_viewer_cad->setupInteractor(ui.vtk_cad->GetInteractor(), ui.vtk_cad->GetRenderWindow());
	m_viewer_cam->setupInteractor(ui.vtk_cad->GetInteractor(), ui.vtk_cad->GetRenderWindow());
	m_viewer_ret->setupInteractor(ui.vtk_cad->GetInteractor(), ui.vtk_cad->GetRenderWindow());
	ui.vtk_cad->update();
	ui.vtk_cam->update();
	ui.vtk_ret->update();
}

Eigen::Matrix4d THandEyeCali3D::tcp2tcp(TPoint6D target, TPoint6D source)
{
	Eigen::Matrix4d ret = Eigen::Matrix4d::Zero();

	//旋转部分
	Eigen::Vector3d euler_target(target.rx, target.ry, target.rz);
	Eigen::Vector3d euler_source(source.rx, source.ry, source.rz);
	Eigen::Matrix3d mat_target;
	Eigen::Matrix3d mat_source;

	//XYZ的旋转顺序
	mat_target = Eigen::AngleAxisd(euler_target(0), Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxisd(euler_target(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(euler_target(2), Eigen::Vector3d::UnitZ());
	mat_source = Eigen::AngleAxisd(euler_source(0), Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxisd(euler_source(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(euler_source(2), Eigen::Vector3d::UnitZ());

	Eigen::Matrix3d rotation = mat_source * mat_target.inverse();
	ret.block(0, 0, 3, 3) = rotation;

	//平移部分
	ret(0, 3) = source.x - target.x;
	ret(1, 3) = source.y - target.y;
	ret(2, 3) = source.z - target.z;

	//最后一行
	ret(3, 0) = 0;
	ret(3, 1) = 0;
	ret(3, 2) = 0;
	ret(3, 3) = 1;

	return ret;
}

void THandEyeCali3D::test()
{
	double tr = 3.1415926 / 180;
	double rx1 = tr * 10;
	double rx2 = tr * 70;
	TPoint6D a(0, 0, 0, 0, 0, rx1);
	TPoint6D b(0, 0, 0, 0, 0, rx2);
	Eigen::Matrix4d ret = tcp2tcp(a, b);
	std::cout << ret << std::endl;
}

void THandEyeCali3D::on_btn_test_clicked()
{
	test();
}

Eigen::Matrix4d THandEyeCali3D::mat2mat(Eigen::Matrix4d target, Eigen::Matrix4d source)
{
	Eigen::Matrix4d ret = Eigen::Matrix4d::Zero();
	ret = source * target.inverse();

	return ret;
}

bool THandEyeCali3D::solveAXXB_1(Poses A_, Poses B_, Pose& X_)
{
	if (A_.size() != B_.size())
	{
		std::cout << "two sizes should be equal" << std::endl;
		return false;
	}
	if (A_.size() < 2)
	{
		std::cout << "at least two motions are needed" << std::endl;
		return false;
	}

	Eigen::MatrixXd m = Eigen::MatrixXd::Zero(12 * A_.size(), 12);
	Eigen::VectorXd b = Eigen::VectorXd::Zero(12 * A_.size());
	for (int i = 0; i < A_.size(); i++)
	{
		//extract R,t from homogophy matrix
		Eigen::Matrix3d Ra = A_[i].topLeftCorner(3, 3);
		Eigen::Vector3d Ta = A_[i].topRightCorner(3, 1);
		Eigen::Matrix3d Rb = B_[i].topLeftCorner(3, 3);
		Eigen::Vector3d Tb = B_[i].topRightCorner(3, 1);

		m.block<9, 9>(12 * i, 0) = Eigen::MatrixXd::Identity(9, 9) - kroneckerProduct(Ra, Rb);
		Eigen::Matrix3d Ta_skew = skew(Ta);
		m.block<3, 9>(12 * i + 9, 0) = kroneckerProduct(Eigen::MatrixXd::Identity(3, 3), Tb.transpose());
		m.block<3, 3>(12 * i + 9, 9) = Eigen::MatrixXd::Identity(3, 3) - Ra;
		b.block<3, 1>(12 * i + 9, 0) = Ta;
	}

	Eigen::Matrix<double, 12, 1> x = m.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	Eigen::Matrix3d R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(x.data()); //row major

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
	X_ = Pose::Identity(4, 4);
	X_.topLeftCorner(3, 3) = svd.matrixU() * svd.matrixV().transpose();
	X_.topRightCorner(3, 1) = x.block<3, 1>(9, 0);
	return true;
}

bool THandEyeCali3D::solveAXXB_2(Poses A_, Poses B_, Pose& X_)
{
	if (A_.size() != B_.size())
	{
		std::cout << "two sizes should be equal" << std::endl;
		return false;
	}
	if (A_.size() < 2)
	{
		std::cout << "at least two motions are needed" << std::endl;
		return false;
	}

	Eigen::MatrixXd m = Eigen::MatrixXd::Zero(9 * A_.size(), 9);
	Eigen::MatrixXd n = Eigen::MatrixXd::Zero(3 * A_.size(), 4);
	for (int i = 0; i < A_.size(); i++)
	{
		//extract R,t from homogophy matrix
		Eigen::Matrix3d Ra = A_[i].topLeftCorner(3, 3);
		Eigen::Vector3d Ta = A_[i].topRightCorner(3, 1);
		Eigen::Matrix3d Rb = B_[i].topLeftCorner(3, 3);
		//    Eigen::Vector3d Tb = B_[i].topRightCorner(3,1);

		m.block<9, 9>(9 * i, 0) = Eigen::MatrixXd::Identity(9, 9) - kroneckerProduct(Ra, Rb);
		n.block<3, 3>(3 * i, 0) = Ra - Eigen::MatrixXd::Identity(3, 3);
		n.block<3, 1>(3 * i, 3) = Ta;
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeFullV | Eigen::ComputeFullU);
	if (!svd.computeV())
		std::cout << "fail to compute V" << std::endl;

	Eigen::Matrix3d R_alpha;
	R_alpha.row(0) = svd.matrixV().block<3, 1>(0, 8).transpose();
	R_alpha.row(1) = svd.matrixV().block<3, 1>(3, 8).transpose();
	R_alpha.row(2) = svd.matrixV().block<3, 1>(6, 8).transpose();
	//double a = std::fabs(R_alpha.determinant());
	double det = R_alpha.determinant();
	double alpha = std::pow(std::abs(det), 4. / 3.) / det;

	Eigen::HouseholderQR<Eigen::Matrix3d> qr(R_alpha / alpha);

	X_ = Pose::Identity(4, 4);
	Eigen::Matrix3d Q = qr.householderQ();
	Eigen::Matrix3d R = Q.transpose() * R_alpha / alpha;
	Eigen::Vector3d R_diagonal = R.diagonal();

	Eigen::Matrix3d Rx;
	for (int i = 0; i < 3; i++)
	{
		Rx.block<3, 1>(0, i) = (R_diagonal(i) >= 0 ? 1 : -1) * Q.col(i);
	}
	X_.topLeftCorner(3, 3) = Rx;

	Eigen::MatrixXd b = Eigen::MatrixXd::Zero(3 * A_.size(), 1);
	for (int i = 0; i < A_.size(); i++)
	{
		Eigen::Vector3d Tb = B_[i].topRightCorner(3, 1);
		b.block<3, 1>(3 * i, 0) = Rx * Tb;
	}

	X_.topRightCorner(4, 1) = n.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	return true;
}

bool THandEyeCali3D::solveAXXB_3(Poses A_, Poses B_, Pose& X_)
{
	if (A_.size() != B_.size())
	{
		std::cout << "two sizes should be equal" << std::endl;
		return false;
	}
	if (A_.size() < 2)
	{
		std::cout << "at least two motions are needed" << std::endl;
		return false;
	}

	Eigen::MatrixXd m = Eigen::MatrixXd::Zero(12 * A_.size(), 12);
	for (int i = 0; i < A_.size(); i++)
	{
		//extract R,t from homogophy matrix
		Eigen::Matrix3d Ra = A_[i].topLeftCorner(3, 3);
		Eigen::Vector3d Ta = A_[i].topRightCorner(3, 1);
		Eigen::Matrix3d Rb = B_[i].topLeftCorner(3, 3);
		Eigen::Vector3d Tb = B_[i].topRightCorner(3, 1);

		m.block<9, 9>(12 * i, 0) = Eigen::MatrixXd::Identity(9, 9) - kroneckerProduct(Ra, Rb);
		Eigen::Matrix3d Ta_skew = skew(Ta);
		m.block<3, 9>(12 * i + 9, 0) = kroneckerProduct(Ta_skew, Tb.transpose());
		m.block<3, 3>(12 * i + 9, 9) = Ta_skew - Ta_skew * Ra;
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeFullV | Eigen::ComputeFullU);
	if (!svd.computeV())
		std::cout << "fail to compute V" << std::endl;

	Eigen::Matrix3d R_alpha;
	R_alpha.row(0) = svd.matrixV().block<3, 1>(0, 11).transpose();
	R_alpha.row(1) = svd.matrixV().block<3, 1>(3, 11).transpose();
	R_alpha.row(2) = svd.matrixV().block<3, 1>(6, 11).transpose();
	//double a = std::fabs(R_alpha.determinant());
	//double alpha = R_alpha.determinant()/(pow(std::fabs(R_alpha.determinant()),4./3.));
	double det = R_alpha.determinant();
	double alpha = std::pow(std::abs(det), 4. / 3.) / det;
	Eigen::HouseholderQR<Eigen::Matrix3d> qr(R_alpha / alpha);

	X_ = Pose::Identity(4, 4);
	Eigen::Matrix3d Q = qr.householderQ();
	Eigen::Matrix3d Rwithscale = alpha * Q.transpose() * R_alpha;
	Eigen::Vector3d R_diagonal = Rwithscale.diagonal();
	for (int i = 0; i < 3; i++)
	{
		X_.block<3, 1>(0, i) = static_cast<int>(R_diagonal(i) >= 0 ? 1 : -1) * Q.col(i);
	}

	X_.topRightCorner(3, 1) = svd.matrixV().block<3, 1>(9, 11) / alpha;

	return true;
}

Eigen::Matrix3d THandEyeCali3D::skew(Eigen::Vector3d u)
{
	Eigen::Matrix3d u_hat = Eigen::MatrixXd::Zero(3, 3);
	u_hat(0, 1) = u(2);
	u_hat(1, 0) = -u(2);
	u_hat(0, 2) = -u(1);
	u_hat(2, 0) = u(1);
	u_hat(1, 2) = u(0);
	u_hat(2, 1) = -u(0);

	return u_hat;
}

//void Tsai_HandEye(Mat& Hcg, vector<Mat> Hgij, vector<Mat> Hcij)
//{
//	CV_Assert(Hgij.size() == Hcij.size());
//	int nStatus = Hgij.size();
//
//	Mat Rgij(3, 3, CV_64FC1);
//	Mat Rcij(3, 3, CV_64FC1);
//
//	Mat rgij(3, 1, CV_64FC1);
//	Mat rcij(3, 1, CV_64FC1);
//
//	double theta_gij;
//	double theta_cij;
//
//	Mat rngij(3, 1, CV_64FC1);
//	Mat rncij(3, 1, CV_64FC1);
//
//	Mat Pgij(3, 1, CV_64FC1);
//	Mat Pcij(3, 1, CV_64FC1);
//
//	Mat tempA(3, 3, CV_64FC1);
//	Mat tempb(3, 1, CV_64FC1);
//
//	Mat A;
//	Mat b;
//	Mat pinA;
//
//	Mat Pcg_prime(3, 1, CV_64FC1);
//	Mat Pcg(3, 1, CV_64FC1);
//	Mat PcgTrs(1, 3, CV_64FC1);
//
//	Mat Rcg(3, 3, CV_64FC1);
//	Mat eyeM = Mat::eye(3, 3, CV_64FC1);
//
//	Mat Tgij(3, 1, CV_64FC1);
//	Mat Tcij(3, 1, CV_64FC1);
//
//	Mat tempAA(3, 3, CV_64FC1);
//	Mat tempbb(3, 1, CV_64FC1);
//
//	Mat AA;
//	Mat bb;
//	Mat pinAA;
//
//	Mat Tcg(3, 1, CV_64FC1);
//
//	for (int i = 0; i < nStatus; i++)
//	{
//		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
//		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
//
//		Rodrigues(Rgij, rgij);
//		Rodrigues(Rcij, rcij);
//
//		theta_gij = norm(rgij);
//		theta_cij = norm(rcij);
//
//		rngij = rgij / theta_gij;
//		rncij = rcij / theta_cij;
//
//		Pgij = 2 * sin(theta_gij / 2) * rngij;
//		Pcij = 2 * sin(theta_cij / 2) * rncij;
//
//		tempA = skew(Pgij + Pcij);
//		tempb = Pcij - Pgij;
//
//		A.push_back(tempA);
//		b.push_back(tempb);
//	}
//
//	//Compute rotation
//	invert(A, pinA, DECOMP_SVD);
//
//	Pcg_prime = pinA * b;
//	Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
//	PcgTrs = Pcg.t();
//	Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg) * norm(Pcg)) * skew(Pcg));
//
//	//Compute Translation 
//	for (int i = 0; i < nStatus; i++)
//	{
//		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
//		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
//		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
//		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);
//
//
//		tempAA = Rgij - eyeM;
//		tempbb = Rcg * Tcij - Tgij;
//
//		AA.push_back(tempAA);
//		bb.push_back(tempbb);
//	}
//
//	invert(AA, pinAA, DECOMP_SVD);
//	Tcg = pinAA * bb;
//
//	Rcg.copyTo(Hcg(Rect(0, 0, 3, 3)));
//	Tcg.copyTo(Hcg(Rect(3, 0, 1, 3)));
//	Hcg.at<double>(3, 0) = 0.0;
//	Hcg.at<double>(3, 1) = 0.0;
//	Hcg.at<double>(3, 2) = 0.0;
//	Hcg.at<double>(3, 3) = 1.0;
//
//}
//
//Mat skew(const Mat A) //Compute the skew symmetric matrix.
//{
//	CV_Assert(A.cols == 1 && A.rows == 3);
//	Mat B(3, 3, CV_64FC1);
//
//	B.at<double>(0, 0) = 0.0;
//	B.at<double>(0, 1) = -A.at<double>(2, 0);
//	B.at<double>(0, 2) = A.at<double>(1, 0);
//
//	B.at<double>(1, 0) = A.at<double>(2, 0);
//	B.at<double>(1, 1) = 0.0;
//	B.at<double>(1, 2) = -A.at<double>(0, 0);
//
//	B.at<double>(2, 0) = -A.at<double>(1, 0);
//	B.at<double>(2, 1) = A.at<double>(0, 0);
//	B.at<double>(2, 2) = 0.0;
//
//	return B;
//}
