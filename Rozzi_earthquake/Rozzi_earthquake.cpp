//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - ChEasyBody objects
//     - collisions and contacts 
//     - imposing a ground-relative motion to a body
//  
//	 CHRONO 
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
   
 
#include "physics/ChSystem.h"
#include "physics/ChBodyEasy.h"
#include "assets/ChTexture.h"
#include "motion_functions/ChFunction_Sine.h"
#include "unit_IRRLICHT/ChIrrApp.h"
#include "physics/ChMaterialSurface.h"


// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::collision;

// Use the main namespaces of Irrlicht
using namespace irr;
         
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui; 





 
ChFunction* create_motion(std::string filename_pos, double t_offset = 0, double factor =1.0)
{
	ChStreamInAsciiFile mstream(GetChronoDataFile(filename_pos).c_str());
	
	ChFunction_Recorder* mrecorder = new ChFunction_Recorder;
	
	while(!mstream.End_of_stream())
	{
		double time = 0;
		double value = 0;
		try
		{
			mstream >> time;
			mstream >> value;

			GetLog() << "  t=" << time << "  p=" << value << "\n";

			mrecorder->AddPoint(time + t_offset, value * factor);
		}
		catch(ChException myerror)
		{
			GetLog() << "  End parsing file " << GetChronoDataFile(filename_pos).c_str() << " because: \n  " << myerror.what() << "\n";
			break;
		}
	}
	GetLog() << "Done parsing. \n";

	return mrecorder;
}


int main(int argc, char* argv[])
{
	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Collisions between objects",core::dimension2d<u32>(800,600),false); //screen dimensions

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(1,1,-5), core::vector3df(3,3,0));		//to change the position of camera
	application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));
 
	// Create a shared material surface used by columns etc.
	ChSharedPtr<ChMaterialSurface> mmat(new ChMaterialSurface);
	mmat->SetFriction(0.57735f);
	//mmat->SetSpinningFriction(0.01f);
	//mmat->SetRollingFriction(0.01f);
	mmat->SetCompliance(0.000004f);
		mmat->SetComplianceT(0.000004f);
		mmat->SetDampingF(0.00176f);
//	mmat->SetRestitution(0.9f);




	// Create all the rigid bodies.

	// Create a floor that is fixed (that is used also to represent the aboslute reference)

	ChSharedPtr<ChBodyEasyBox> floorBody(new ChBodyEasyBox( 20,2,20,  3000,	false, true));		//to create the floor, false -> doesn't represent a collide's surface
	floorBody->SetPos( ChVector<>(0,-2,0) );
	floorBody->SetBodyFixed(true);		//SetBodyFixed(true) -> it's fixed, it doesn't move respect to the Global Position System

	mphysicalSystem.Add(floorBody);

	// optional, attach a texture for better visualization
	ChSharedPtr<ChTexture> mtexture(new ChTexture());
    mtexture->SetTextureFilename(GetChronoDataFile("blu.png"));		//texture in /data
	floorBody->AddAsset(mtexture);		//add texture to the system



	// Create the table that is subject to earthquake

	ChSharedPtr<ChBodyEasyBox> tableBody(new ChBodyEasyBox( 17,1,15,  3000,	true, true));
	tableBody->SetPos( ChVector<>(0,-0.5,0) );

	mphysicalSystem.Add(tableBody);

	// optional, attach a texture for better visualization
	ChSharedPtr<ChTexture> mtextureconcrete(new ChTexture());
    mtextureconcrete->SetTextureFilename(GetChronoDataFile("grass.png"));
	tableBody->AddAsset(mtextureconcrete);


	// Create the constraint between ground and table. If no earthquake, it just
	// keeps the table in position.

	ChSharedPtr<ChLinkLockLock> linkEarthquake(new ChLinkLockLock);
	linkEarthquake->Initialize(tableBody, floorBody, ChCoordsys<>(ChVector<>(0,0,0)) );

//	double time_offset = 0; // begin earthquake after 5 s to allow stabilization of blocks after creation.
//	double ampl_factor = 1; // use lower or greater to scale the earthquake.
//	bool   use_barrier = false; // if true, the Barrier data files are used, otherwise the No_Barrier datafiles are used

	// Define the horizontal motion, on x:
	ChFunction_Sine* mmotion_x = new ChFunction_Sine(0,0,0); // phase freq ampl, carachteristics of input motion
	linkEarthquake->SetMotion_X(mmotion_x);
//	ChFunction* mmotion_x    = create_motion("accelerogrammi/acc1.txt", time_offset, ampl_factor);
//	linkEarthquake->SetMotion_X(mmotion_x);
//	ChFunction* mmotion_x_NB = create_motion("Time history 10x0.50 Foam (d=6 m)/No_Barrier_Uh.txt", time_offset, ampl_factor);

	// Define the vertical motion, on y:
//	ChFunction* mmotion_y    = create_motion("Time history 10x0.50 Foam (d=6 m)/Barrier_Uv.txt", time_offset, ampl_factor);
//	ChFunction* mmotion_y_NB = create_motion("Time history 10x0.50 Foam (d=6 m)/No_Barrier_Uv.txt", time_offset, ampl_factor);

/*	if (use_barrier)
	{
		linkEarthquake->SetMotion_Z(mmotion_x);
		linkEarthquake->SetMotion_Y(mmotion_y);
	}
	else
	{
		linkEarthquake->SetMotion_Z(mmotion_x_NB);
		linkEarthquake->SetMotion_Y(mmotion_y_NB);
	}
*/

	mphysicalSystem.Add(linkEarthquake);


	// Pointers to some objects that will be plotted, for future use.
	ChSharedPtr<ChBody> plot_brick_1;
	ChSharedPtr<ChBody> plot_brick_3;
	ChSharedPtr<ChBody> plot_brick_4;

//	ChSharedPtr<ChBody> plot_brick_2;
	ChSharedPtr<ChBody> plot_table;
/*
	ChSharedPtr<ChBody> plot_brick_5;
	ChSharedPtr<ChBody> plot_brick_6;
	ChSharedPtr<ChBody> plot_brick_7;
	ChSharedPtr<ChBody> plot_brick_8;
	ChSharedPtr<ChBody> plot_brick_9;
	ChSharedPtr<ChBody> plot_brick_10;
	ChSharedPtr<ChBody> plot_brick_11;
	ChSharedPtr<ChBody> plot_brick_12;
	ChSharedPtr<ChBody> plot_brick_13;
*/
	plot_table = tableBody; // others will be hooked later.


	// Create the elements of the model


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// BLOCCO SINGOLO

//		double spacing = 2.2;
		double density = 2670;
//		int nedges=10;
        double dimx = 0.5;
		double dimy = 1;
		double dimz = 0.5;


		

	ChSharedPtr<ChBodyEasyBox> mattone1(new ChBodyEasyBox(
			dimx, dimy , dimz, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_mattone1(ChVector<>(0, (dimy/2),0) ,ChQuaternion<>(cos(2*CH_C_DEG_TO_RAD),0,0,sin(2*CH_C_DEG_TO_RAD)));
		mattone1->SetCoord(cog_mattone1);
		mattone1->SetMaterialSurface(mmat);
		plot_brick_1 = mattone1;
		plot_brick_3 = mattone1;
		plot_brick_4 = mattone1;

		mphysicalSystem.Add(mattone1);

		//create a texture for the mattone1
		ChSharedPtr<ChTexture> mtexturemattone1(new ChTexture());
		mtexturemattone1->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		mattone1->AddAsset(mtexturemattone1);


		//to create mattone2
/*
				ChSharedPtr<ChBodyEasyBox> mattone2(new ChBodyEasyBox(
			0.4, 2 , 0.4, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_mattone2(ChVector<>(0, 3, 0));
		mattone2->SetCoord(cog_mattone2);
		plot_brick_2 = mattone2;

		mphysicalSystem.Add(mattone2);

		//create a texture for the topBeam
		ChSharedPtr<ChTexture> mtexturemattone2(new ChTexture());
		mtexturemattone2->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		mattone2->AddAsset(mtexturemattone2);
*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


/*
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			// BLOCCHI IMPILATI

//		double spacing = 2.2;
		double density = 2670;
//		int nedges=10;
        double dimx = 0.17;
		double dimy = 1;
		double dimz = 0.502;


	for (int ai = 0; ai < 4; ai++)  // N. of walls
	{ 		

	ChSharedPtr<ChBodyEasyBox> mattone1(new ChBodyEasyBox(
			dimx, dimy , dimz, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_mattone1(ChVector<>(0, (dimy/2+dimy*ai), 0),ChQuaternion<>(0, 0, 0, 0));
		mattone1->SetCoord(cog_mattone1);
		mattone1->SetMaterialSurface(mmat);
		
		mphysicalSystem.Add(mattone1);

		//create a texture for the mattone1
		ChSharedPtr<ChTexture> mtexturemattone1(new ChTexture());
		mtexturemattone1->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		mattone1->AddAsset(mtexturemattone1);

			if (ai == 3)
		{

		plot_brick_1 = mattone1;
		plot_brick_3 = mattone1;
		plot_brick_4 = mattone1;
			}

	}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

/*
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// TRILITE



//		double spacing = 2.2;
		double density = 2670;
//		int nedges=10;
 
		//COLONNE

		double dimx = 0.22;
		double dimy = 0.8;
		double dimz = 0.65;

		//colonna1

	ChSharedPtr<ChBodyEasyBox> colonna1(new ChBodyEasyBox(
			dimx, dimy , dimz, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_colonna1(ChVector<>((dimx/2), (dimy/2), (dimz/2)),ChQuaternion<>(0, 0, 0, 0));
		colonna1->SetCoord(cog_colonna1);
		colonna1->SetMaterialSurface(mmat);
		plot_brick_5 = colonna1;
		plot_brick_6 = colonna1;
		plot_brick_7 = colonna1;
		mphysicalSystem.Add(colonna1);

		//create a texture for the colonna1
		ChSharedPtr<ChTexture> mtexturecolonna1(new ChTexture());
		mtexturecolonna1->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		colonna1->AddAsset(mtexturecolonna1);


		//colonna2

ChSharedPtr<ChBodyEasyBox> colonna2(new ChBodyEasyBox(
			dimx, dimy , dimz, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_colonna2(ChVector<>((1.02-(dimx/2)), (dimy/2), (dimz/2)),ChQuaternion<>(0, 0, 0, 0));
		colonna2->SetCoord(cog_colonna2);
		colonna2->SetMaterialSurface(mmat);
		plot_brick_8 = colonna2;
		plot_brick_9 = colonna2;
		plot_brick_10 = colonna2;
		mphysicalSystem.Add(colonna2);

		//create a texture for the colonna2
		ChSharedPtr<ChTexture> mtexturecolonna2(new ChTexture());
		mtexturecolonna2->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		colonna2->AddAsset(mtexturecolonna2);

		//TRAVE

		double travex = 1.02 ;
		double travey = 0.15 ;
		double travez = 0.65 ;

     ChSharedPtr<ChBodyEasyBox> trave(new ChBodyEasyBox(
			travex, travey , travez, // x y z sizes
			density,
			true,
			true));

		ChCoordsys<> cog_trave(ChVector<>((travex/2), (dimy+(travey/2)), (travez/2)),ChQuaternion<>(0, 0, 0, 0));
		trave->SetCoord(cog_trave);
		trave->SetMaterialSurface(mmat);
		plot_brick_11 = trave;
		plot_brick_12 = trave;
		plot_brick_13 = trave;
		mphysicalSystem.Add(trave);

		//create a texture for the trave
		ChSharedPtr<ChTexture> mtexturetrave(new ChTexture());
		mtexturetrave->SetTextureFilename(GetChronoDataFile("whiteconcrete.jpg"));
		trave->AddAsset(mtexturetrave);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/


	// Use this function for adding a ChIrrNodeAsset to all items
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();

	// Use this function for 'converting' assets into Irrlicht meshes 
	application.AssetUpdateAll();

	// This is to enable shadow maps (shadow casting with soft shadows) in Irrlicht
	// for all objects (or use application.AddShadow(..) for enable shadow on a per-item basis)

	application.AddShadowAll();


	// Modify some setting of the physical system for the simulation, if you want
//	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN); // slower but more pricise
//	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD); // VELOCE
//	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SYMMSOR);
//	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_JACOBI);
//	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES);
//	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PCG);
//	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_APGD);
//	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_DEM);
//	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);
//	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_SIMPLEX);
	mphysicalSystem.SetIterLCPmaxItersSpeed(80);
	mphysicalSystem.SetIterLCPmaxItersStab(5);
//	mphysicalSystem.SetMaxPenetrationRecoverySpeed(0.8);
//	mphysicalSystem.SetMinBounceSpeed(0.0001);
	
	ChCollisionModel::SetDefaultSuggestedEnvelope(0.005);
	ChCollisionModel::SetDefaultSuggestedMargin  (0.005);

	// scelta del metodo d'integrazione

	mphysicalSystem.SetIntegrationType(ChSystem::INT_ANITESCU);


	//mphysicalSystem.SetUseSleeping(true);

	application.SetStepManage(true);
	application.SetTimestep(0.00001);
	application.SetTryRealtime(false);


	// Files for output data
	ChStreamOutAsciiFile data_earthquake_x("data_earthquake_x.txt");
//	ChStreamOutAsciiFile data_earthquake_y("data_earthquake_y.txt");
//	ChStreamOutAsciiFile data_earthquake_x_NB("data_earthquake_x_NB.txt");
//	ChStreamOutAsciiFile data_earthquake_y_NB("data_earthquake_y_NB.txt");
	ChStreamOutAsciiFile data_table("data_table.txt");
	ChStreamOutAsciiFile data_brick_1("data_brick_1.txt");
	ChStreamOutAsciiFile data_brick_3("data_brick_3.txt");
	ChStreamOutAsciiFile data_brick_4("data_brick_4.txt");

/*
	ChStreamOutAsciiFile data_brick_5("data_brick_5.txt");
	ChStreamOutAsciiFile data_brick_6("data_brick_6.txt");
	ChStreamOutAsciiFile data_brick_7("data_brick_7.txt");
	ChStreamOutAsciiFile data_brick_8("data_brick_8.txt");
	ChStreamOutAsciiFile data_brick_9("data_brick_9.txt");
	ChStreamOutAsciiFile data_brick_10("data_brick_10.txt");
	ChStreamOutAsciiFile data_brick_11("data_brick_11.txt");
	ChStreamOutAsciiFile data_brick_12("data_brick_12.txt");
	ChStreamOutAsciiFile data_brick_13("data_brick_13.txt");
	*/

//	ChStreamOutAsciiFile data_brick_2("data_brick_2.txt");


	// 
	// THE SOFT-REAL-TIME CYCLE
	//
	ChVector<> brick_initial_displacement;

	while (application.GetDevice()->run())
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

		application.DrawAll();

		application.DoStep();

		// save data for plotting
		double time = mphysicalSystem.GetChTime();

/*		if (time <1.5)
			brick_initial_displacement = plot_brick_2->GetPos() - plot_table->GetPos();
		
		if (time >1.5)  // save only after tot seconds to avoid plotting initial settlement
*/		{
			data_earthquake_x << time << " " 
							  << mmotion_x->Get_y(time) << "\n";
//							  << mmotion_x->Get_y_dx(time) << " "
//							  << mmotion_x->Get_y_dxdx(time) << "\n";

/*			data_earthquake_y << time << " " 
							  << mmotion_y->Get_y(time) << " "
							  << mmotion_y->Get_y_dx(time) << " "
							  << mmotion_y->Get_y_dxdx(time) << "\n";

			data_earthquake_x_NB << time << " " 
							  << mmotion_x_NB->Get_y(time) << " "
							  << mmotion_x_NB->Get_y_dx(time) << " "
							  << mmotion_x_NB->Get_y_dxdx(time) << "\n";

			data_earthquake_y_NB << time << " " 
							  << mmotion_y_NB->Get_y(time) << " "
							  << mmotion_y_NB->Get_y_dx(time) << " "
							  << mmotion_y_NB->Get_y_dxdx(time) << "\n";
*/							  

			data_table	<< mphysicalSystem.GetChTime() << " " 
						<< plot_table->GetPos().x  << "\n";  // because created at x=4.05, and we want to plot from 0
/*						<< plot_table->GetPos().y +0.5 << " "
						<< plot_table->GetPos().z << " "
						<< plot_table->GetPos_dt().x << " "
						<< plot_table->GetPos_dt().y << " "
						<< plot_table->GetPos_dt().z << " "
						<< plot_table->GetPos_dtdt().x << " "
						<< plot_table->GetPos_dtdt().y << " "
						<< plot_table->GetPos_dtdt().z << "\n";
*/

			ChFrameMoving<> rel_motion;
			plot_table->TransformParentToLocal(plot_brick_1->GetFrame_REF_to_abs(), rel_motion);

			data_brick_1 << mphysicalSystem.GetChTime() << " " 
						<< rel_motion.GetPos().x  << " "
						<< rel_motion.GetPos().y -0.5 << "\n";
//						<< rel_motion.GetPos().z  << "\n";
/*						<< rel_motion.GetPos_dt().x << " "
						<< rel_motion.GetPos_dt().y << " "
						<< rel_motion.GetPos_dt().z << " "
						<< rel_motion.GetPos_dtdt().x << " "
						<< rel_motion.GetPos_dtdt().y << " "
						<< rel_motion.GetPos_dtdt().z << "\n";
*/


			ChFrameMoving<> rel_motion_3;
			plot_table->TransformParentToLocal(plot_brick_3->GetFrame_REF_to_abs(), rel_motion_3);

			data_brick_3 << mphysicalSystem.GetChTime() << " "				         
						 << rel_motion_3.GetRotAxis().z << " "  
             << rel_motion_3.GetRotAngle() << "\n";



/*						ChFrameMoving<> rel_motion_4;
			plot_table->TransformParentToLocal(plot_brick_4->GetFrame_REF_to_abs(), rel_motion_4);

			data_brick_4 << mphysicalSystem.GetChTime() << " "				         
           << rel_motion_4.GetRotAxis().z << "\n";
*/

/*
// OUTPUT TRILITE ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//colonna1
						ChFrameMoving<> rel_motion_5;
			plot_table->TransformParentToLocal(plot_brick_5->GetFrame_REF_to_abs(), rel_motion_5);

			data_brick_5 << mphysicalSystem.GetChTime() << " " 
						<< rel_motion_5.GetPos().x +4.05 << " "
						<< rel_motion_5.GetPos().y -0.5 << "\n";
//						<< rel_motion_5.GetPos().z  << "\n";
//						<< rel_motion_5.GetPos_dt().x << " "
//						<< rel_motion_5.GetPos_dt().y << " "
//						<< rel_motion_5.GetPos_dt().z << " "
//						<< rel_motion_5.GetPos_dtdt().x << " "
//						<< rel_motion_5.GetPos_dtdt().y << " "
//						<< rel_motion_5.GetPos_dtdt().z << "\n";


						ChFrameMoving<> rel_motion_6;
			plot_table->TransformParentToLocal(plot_brick_6->GetFrame_REF_to_abs(), rel_motion_6);

			data_brick_6 << mphysicalSystem.GetChTime() << " "				         
//						 << rel_motion_6.GetRotAxis() << " "  
             << rel_motion_6.GetRotAngle() << "\n";
 

						ChFrameMoving<> rel_motion_7;
			plot_table->TransformParentToLocal(plot_brick_7->GetFrame_REF_to_abs(), rel_motion_7);

			data_brick_7 << mphysicalSystem.GetChTime() << " "				         
						 << rel_motion_7.GetRotAxis().z << "\n"; 

//colonna2

									ChFrameMoving<> rel_motion_8;
			plot_table->TransformParentToLocal(plot_brick_8->GetFrame_REF_to_abs(), rel_motion_8);

			data_brick_8 << mphysicalSystem.GetChTime() << " " 
						<< rel_motion_8.GetPos().x +4.05 << " "
						<< rel_motion_8.GetPos().y -0.5 << "\n";
//						<< rel_motion_8.GetPos().z  << "\n";
//						<< rel_motion_8.GetPos_dt().x << " "
	//					<< rel_motion_8.GetPos_dt().y << " "
	//					<< rel_motion_8.GetPos_dt().z << " "
	//					<< rel_motion_8.GetPos_dtdt().x << " "
	//					<< rel_motion_8.GetPos_dtdt().y << " "
	//					<< rel_motion_8.GetPos_dtdt().z << "\n";


						ChFrameMoving<> rel_motion_9;
			plot_table->TransformParentToLocal(plot_brick_9->GetFrame_REF_to_abs(), rel_motion_9);

			data_brick_9 << mphysicalSystem.GetChTime() << " "				         
//						 << rel_motion_9.GetRotAxis() << " "  
             << rel_motion_9.GetRotAngle() << "\n";
 

						ChFrameMoving<> rel_motion_10;
			plot_table->TransformParentToLocal(plot_brick_10->GetFrame_REF_to_abs(), rel_motion_10);

			data_brick_10 << mphysicalSystem.GetChTime() << " "				         
						 << rel_motion_10.GetRotAxis().z << "\n"; 

//trave

									ChFrameMoving<> rel_motion_11;
			plot_table->TransformParentToLocal(plot_brick_11->GetFrame_REF_to_abs(), rel_motion_11);

			data_brick_11 << mphysicalSystem.GetChTime() << " " 
						<< rel_motion_11.GetPos().x +4.05 << " "
						<< rel_motion_11.GetPos().y -0.5 << "\n";
//						<< rel_motion_11.GetPos().z  << "\n";
//						<< rel_motion_11.GetPos_dt().x << " "
   //					<< rel_motion_11.GetPos_dt().y << " "
//						<< rel_motion_11.GetPos_dt().z << " "
//						<< rel_motion_11.GetPos_dtdt().x << " "
//						<< rel_motion_11.GetPos_dtdt().y << " "
//						<< rel_motion_11.GetPos_dtdt().z << "\n";


						ChFrameMoving<> rel_motion_12;
			plot_table->TransformParentToLocal(plot_brick_12->GetFrame_REF_to_abs(), rel_motion_12);

			data_brick_12 << mphysicalSystem.GetChTime() << " "				         
//						 << rel_motion_12.GetRotAxis() << " "  
             << rel_motion_12.GetRotAngle() << "\n";
 

						ChFrameMoving<> rel_motion_13;
			plot_table->TransformParentToLocal(plot_brick_13->GetFrame_REF_to_abs(), rel_motion_13);

			data_brick_13 << mphysicalSystem.GetChTime() << " "				         
						 << rel_motion_13.GetRotAxis().z << "\n"; 

//   FINE OUTPUT TRILITE +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			*/


/*			ChFrameMoving<> rel_motion_2;
			plot_table->TransformParentToLocal(plot_brick_2->GetFrame_REF_to_abs(), rel_motion_2);

			data_brick_2 << mphysicalSystem.GetChTime() << " "
				<< rel_motion_2.GetPos().x - brick_initial_displacement.x << " "
				<< rel_motion_2.GetPos().y - brick_initial_displacement.y << " "
				<< rel_motion_2.GetPos().z - brick_initial_displacement.z << " "
				<< rel_motion_2.GetPos_dt().x << " "
				<< rel_motion_2.GetPos_dt().y << " "
				<< rel_motion_2.GetPos_dt().z << " "
				<< rel_motion_2.GetPos_dtdt().x << " "
				<< rel_motion_2.GetPos_dtdt().y << " "
				<< rel_motion_2.GetPos_dtdt().z << "\n";
*/			
			// end plotting data logout
		}

		application.GetVideoDriver()->endScene();

		// Exit simulation if time greater than ..
		if (mphysicalSystem.GetChTime() > 3) 
			break;
	}


	// optional: automate the plotting launching GNUplot with a commandfile

	/*bool use_gnuplot = true;

	if (use_gnuplot)
	{
		ChStreamOutAsciiFile gnuplot_command("__data.gpl");
		gnuplot_command << "set term wxt 0 \n";
		gnuplot_command << "plot \"data_earthquake_x.dat\" using 1:2 with lines title \"x, barrier\" ,";
		gnuplot_command << "     \"data_earthquake_x_NB.dat\" using 1:2 with lines  title \"x, no barrier\" ,";
		gnuplot_command << "     \"data_earthquake_y.dat\" using 1:2 with lines title \"y, barrier\" ,";
		gnuplot_command << "     \"data_earthquake_y_NB.dat\" using 1:2 with lines  title \"y, no barrier\" ,";
		gnuplot_command << "     \"data_table.dat\" using 1:2 every 10 pt 7 ps 0.2  title \"x, table\" ,";
		gnuplot_command << "     \"data_table.dat\" using 1:3 every 10 pt 7 ps 0.2  title \"y, table\" \n";
		gnuplot_command << "set xlabel \"Time (s)\" \n";
		gnuplot_command << "set ylabel \"x (m)\" \n";
		gnuplot_command << "set grid \n";
				
		gnuplot_command << "set term wxt 1 \n";
		gnuplot_command << "plot \"data_earthquake_x.dat\" using 1:3 with lines title \"Vx, barrier\" ,";
		gnuplot_command << "     \"data_earthquake_x_NB.dat\" using 1:3 with lines  title \"Vx, no barrier\" ,";
		gnuplot_command << "     \"data_table.dat\" using 1:5 every 5  pt 7 ps 0.2 title \"Vx, table\"  \n";
		gnuplot_command << "set xlabel \"Time (s)\" \n";
		gnuplot_command << "set ylabel \"v_x (m/s)\" \n";
		gnuplot_command << "set grid \n";

		gnuplot_command << "set term wxt 2 \n";
		gnuplot_command << "plot \"data_earthquake_y.dat\" using 1:3 with lines title \"Vy, barrier\" ,";
		gnuplot_command << "     \"data_earthquake_y_NB.dat\" using 1:3 with lines  title \"Vy, no barrier\" ,";
		gnuplot_command << "     \"data_table.dat\" using 1:6 every 5  pt 7 ps 0.2 title \"Vy, table\" \n";
		gnuplot_command << "set xlabel \"Time (s)\" \n";
		gnuplot_command << "set ylabel \"v_x (m/s)\" \n";
		gnuplot_command << "set grid \n";

		gnuplot_command << "set term wxt 3 \n";
		gnuplot_command << "plot \"data_earthquake_x.dat\" using 1:4 with lines title \"Ax, barrier\", ";
		gnuplot_command << "     \"data_earthquake_x_NB.dat\" using 1:4 with lines title \"Ax, no barrier\" ,";
		gnuplot_command << "     \"data_table.dat\" using 1:8 every 5  pt 7 ps 0.2 title \"Ax, table\" \n";
		gnuplot_command << "set xlabel \"Time (s)\" \n";
		gnuplot_command << "set ylabel \"a_x (m/s^2)\" \n";
		gnuplot_command << "set grid \n";

		gnuplot_command << "set term wxt 4 \n";
		gnuplot_command << "plot \"data_earthquake_y.dat\" using 1:4 with lines title \"Ay, barrier\", ";
		gnuplot_command << "     \"data_earthquake_y_NB.dat\" using 1:4 with lines title \"Ay, no barrier\" ,";
		gnuplot_command << "     \"data_table.dat\" using 1:9 every 5  pt 7 ps 0.2 title \"Ay, table\"  \n";
		gnuplot_command << "set xlabel \"Time (s)\" \n";
		gnuplot_command << "set ylabel \"a_x (m/s^2)\" \n";
		gnuplot_command << "set grid \n";

		gnuplot_command << "set term wxt 5 \n";
		gnuplot_command << "plot \"data_brick_1.dat\" using 1:2 with lines title \"x tip rel. displacement\", ";
		gnuplot_command << "     \"data_brick_1.dat\" using 1:3 with lines title \"y tip rel. displacement\", ";
		gnuplot_command << "     \"data_brick_1.dat\" using 1:4 with lines title \"z tip rel. displacement\"  \n";
		gnuplot_command << "set xlabel \"Time (s)\" \n";
		gnuplot_command << "set ylabel \"(m)\" \n";
		gnuplot_command << "set grid \n";

		gnuplot_command << "set term wxt 6 \n";
		gnuplot_command << "plot \"data_brick_1.dat\" using 1:5 with lines title \"Vx tip rel. speed\", ";
		gnuplot_command << "     \"data_brick_1.dat\" using 1:6 with lines title \"Vy tip rel. speed\", ";
		gnuplot_command << "     \"data_brick_1.dat\" using 1:7 with lines title \"Vz tip rel. speed\"  \n";
		gnuplot_command << "set xlabel \"Time (s)\" \n";
		gnuplot_command << "set ylabel \"(m/s)\" \n";
		gnuplot_command << "set grid \n";

		gnuplot_command << "set term wxt 7 \n";
		gnuplot_command << "plot \"data_brick_1.dat\" using 1:8 with lines title \"Ax body rel. acc.\", ";
		gnuplot_command << "     \"data_brick_1.dat\" using 1:9 with lines title \"Ay body rel. acc.\", ";
		gnuplot_command << "     \"data_brick_1.dat\" using 1:10 with lines title \"Az body rel. acc.\"  \n";
		gnuplot_command << "set xlabel \"Time (s)\" \n";
		gnuplot_command << "set ylabel \"(m/s^2)\" \n";
		gnuplot_command << "set grid \n";

		system ("start gnuplot \"__data.gpl\" -persist");
	}


	//system ("pause");*/

	return 0;
}
  
