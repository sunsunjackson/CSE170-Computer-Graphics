
# include "my_viewer.h"

# include <sigogl/ui_button.h>
# include <sigogl/ui_radio_button.h>
# include <sig/sn_primitive.h>
# include <sig/sn_transform.h>
# include <sig/sn_manipulator.h>

# include <sigogl/ws_run.h>

SnGroup* gCity;
SnGroup* gFalcon;
SnGroup* g;
SnTransform* tCity;
SnTransform* tFalcon;
float gtime;

MyViewer::MyViewer ( int x, int y, int w, int h, const char* l ) : WsViewer(x,y,w,h,l)
{
	_nbut=0;
	_animating=false;
	background(GsColor::black);
	build_ui ();
	build_scene ();
}

void MyViewer::build_ui ()
{
	UiPanel *p, *sp;
	UiManager* uim = WsWindow::uim();
	p = uim->add_panel ( "", UiPanel::HorizLeft );
	p->add ( new UiButton ( "View", sp=new UiPanel() ) );
	{	UiPanel* p=sp;
		p->add ( _nbut=new UiCheckButton ( "Normals", EvNormals ) ); 
	}
	p->add ( new UiButton ( "Animate", EvAnimate ) );
	p->add ( new UiButton ( "Exit", EvExit ) ); p->top()->separate();
}

void MyViewer::add_model ( SnShape* s, GsVec p )
{
	// This method demonstrates how to add some elements to our scene graph: lines,
	// and a shape, and all in a group under a SnManipulator.
	// Therefore we are also demonstrating the use of a manipulator to allow the user to
	// change the position of the object with the mouse. If you do not need mouse interaction,
	// you can just use a SnTransform to apply a transformation instead of a SnManipulator.
	// You would then add the transform as 1st element of the group, and set g->separator(true).
	// Your scene graph should always be carefully designed according to your application needs.

	SnManipulator* manip = new SnManipulator;
	GsMat m;
	m.translation ( p );
	manip->initial_mat ( m );

	SnGroup* g = new SnGroup;
	SnLines* l = new SnLines;
	l->color(GsColor::orange);
	g->add(s);
	g->add(l);
	manip->child(g);
	// manip->visible(false); // call this to turn off mouse interaction

	rootg()->add(manip);
}

static int factorial(int n) {
	if (n == 0) 
		return 1;
	else return (n * factorial(n - 1));
}

static GsPnt bezier(float t, GsArray<GsPnt>& P) {
	GsPnt point;
	int n = P.size() - 1;
	for (int i = 0; i < P.size(); i++) {
		int C = factorial(n) / (factorial(i) * factorial(n - i));
		point += P[i] * float(C) * powf((1.0f - t), float(n - i)) * powf(t, (float)i);
	}
	return point;
}

static GsPnt CRSplines(float t, int i, GsArray<GsPnt>& P) {
	GsPnt I1 = (P[i + 1] - P[i - 1]) / 2;
	GsPnt I2 = (P[i + 2] - P[i]) / 2;
	GsPnt Pp = P[i] + (I1 / 3);
	GsPnt Pm = P[i + 1] + (I2 / 3);
	
	GsArray<GsPnt> temp; 
	temp.push() = P[i];
	temp.push() = Pp;
	temp.push() = Pm;
	temp.push() = P[i + 1];

	return bezier(t, P);
}

void MyViewer::build_scene ()
{

	GsMat trans;

	gCity = new SnGroup;
	gFalcon = new SnGroup;
	g = new SnGroup;

	SnModel* mCity = new SnModel;
	//rpX = 5.0f; rpY = -4.5f; rpZ = -3.5f;
	if (!mCity->model()->load("C:/Users/Lopez/OneDrive/Desktop/CSE/CSE170/FinalProject/sigapp/vs2019/city/sirus City.obj")) {
		gsout << "ha sucker" << gsnl;
		
	}
	mCity->model()->scale(0.5f);
	//rightPeddle->model()->centralize();
	gCity->separator(true);
	gCity->add(tCity = new SnTransform);
	gCity->add(mCity);
	//trans.translation(GsVec(0, 0.0f, 0));
	//tCity->set(trans);	

	SnModel* mFalcon = new SnModel;
	gsout << "check" << gsnl;
	//rpX = 5.0f; rpY = -4.5f; rpZ = -3.5f;
	if (!mFalcon->model()->load("C:/Users/Lopez/OneDrive/Desktop/CSE/CSE170/FinalProject/sigapp/vs2019/falcon/millenium-falcon.obj")) {
		gsout << "ha sucker" << gsnl;
	}
	gsout << "check 2" << gsnl;
	mFalcon->model()->scale(0.10f);
	//mFalcon->model()->centralize();
	gFalcon->separator(true);
	gFalcon->add(tFalcon = new SnTransform);
	
	gFalcon->add(mFalcon);
	trans.translation(GsVec(0.0f, 250.0f, 0.0f));
	tFalcon->set(trans);

	gsout << "hello world";

	g->add(gFalcon);
	g->add(gCity);
	rootg()->add(g);


	GsArray<GsPnt> falconpnt;
	//falconpnt.push() = GsPnt(250.f, 250.f, 0.0f);
	//falconpnt.push() = GsPnt(-250.f, 250.f, 0.0f);
	//falconpnt.push() = GsPnt(-250.f, 250.f, 250.0f);
	//falconpnt.push() = GsPnt(250.f, 250.f, -250.0f);

	float r = 500.f;
	for (float theta = 0.0f; theta < gs2pi; theta += gs2pi /12.0f) {
		falconpnt.push() = GsPnt(r*cosf(theta), 250.f, r*sinf(theta));
	}

	showMyPoints(falconpnt); // shows the points created.
	float deltat = 0.001f;
	for (int i = 1; i < falconpnt.size() - 2; i ++) {
		for(float t = 0.0f; t < 1.0f ; t+= deltat) {
			GsPnt P = CRSplines(t, i, falconpnt); // use CR splines
			falconPath.push() = P; // Creates the path
		}
	}

}

void MyViewer::showMyPoints(GsArray<GsPnt> P) {
	SnModel* sPoints;
	for (int i = 0; i < P.size(); i++) {
		sPoints = new SnModel;
		sPoints->model()->make_sphere(P[i], 20.0f, 50, true);
		sPoints->color(GsColor::red);
		rootg()->add(sPoints);
	}
}

// Below is an example of how to control the main loop of an animation:
void MyViewer::run_animation ()
{
	//if ( _animating ) return; // avoid recursive calls
	//_animating = true;
	//
	//int ind = gs_random ( 0, rootg()->size()-1 ); // pick one child
	//SnManipulator* manip = rootg()->get<SnManipulator>(ind); // access one of the manipulators
	//GsMat m = manip->mat();

	//double frdt = 1.0/30.0; // delta time to reach given number of frames per second
	//double v = 4; // target velocity is 1 unit per second
	//double t=0, lt=0, t0=gs_time();
	//do // run for a while:
	//{	while ( t-lt<frdt ) { ws_check(); t=gs_time()-t0; } // wait until it is time for next frame
	//	double yinc = (t-lt)*v;
	//	if ( t>2 ) yinc=-yinc; // after 2 secs: go down
	//	lt = t;
	//	m.e24 += (float)yinc;
	//	if ( m.e24<0 ) m.e24=0; // make sure it does not go below 0
	//	manip->initial_mat ( m );
	//	render(); // notify it needs redraw
	//	ws_check(); // redraw now
	//}	while ( m.e24>0 );
	//_animating = false;

	double frdt = 1.0 / 60.0;
	double t = 0, lt = 0, t0 = gs_time();
	int i = 0;
	do {
		while (t - lt < frdt) { ws_check(); t = gs_time() - t0; }
		if (i < falconPath.size()) {
			tFalcon->get().translation(falconPath[i]);
		}
		else {
			i = 0;
		}
		i++;
		lt = t;
		render(); // notify it needs redraw
		ws_check(); // redraw now
	} while (true);
	

}

void MyViewer::show_normals ( bool view )
{
	// Note that primitives are only converted to meshes in GsModel
	// at the first draw call.
	GsArray<GsVec> fn;
	SnGroup* r = (SnGroup*)root();
	for ( int k=0; k<r->size(); k++ )
	{	SnManipulator* manip = r->get<SnManipulator>(k);
		SnShape* s = manip->child<SnGroup>()->get<SnShape>(0);
		SnLines* l = manip->child<SnGroup>()->get<SnLines>(1);
		if ( !view ) { l->visible(false); continue; }
		l->visible ( true );
		if ( !l->empty() ) continue; // build only once
		l->init();
		if ( s->instance_name()==SnPrimitive::class_name )
		{	GsModel& m = *((SnModel*)s)->model();
			m.get_normals_per_face ( fn );
			const GsVec* n = fn.pt();
			float f = 0.33f;
			for ( int i=0; i<m.F.size(); i++ )
			{	const GsVec& a=m.V[m.F[i].a]; l->push ( a, a+(*n++)*f );
				const GsVec& b=m.V[m.F[i].b]; l->push ( b, b+(*n++)*f );
				const GsVec& c=m.V[m.F[i].c]; l->push ( c, c+(*n++)*f );
			}
		}  
	}
}

int MyViewer::handle_keyboard ( const GsEvent &e )
{
	int ret = WsViewer::handle_keyboard ( e ); // 1st let system check events
	if ( ret ) return ret;

	double frdt = 1.0 / 30.0;
	double t, lt, t0 = gs_time();
	gtime = 0.0f;
	t = 0;
	lt = 0;

	switch ( e.key )
	{	case GsEvent::KeyEsc : gs_exit(); return 1;
		case 'n' : { bool b=!_nbut->value(); _nbut->value(b); show_normals(b); return 1; }
		case ' ': {
			camera().eye.x += 10.0f;
			camera().center;
			ws_check();
			render();
			break;
		}
		default: gsout<<"Key pressed: "<<e.key<<gsnl;
	}

	return 0;
}

int MyViewer::uievent ( int e )
{
	switch ( e )
	{	case EvNormals: show_normals(_nbut->value()); return 1;
		case EvAnimate: run_animation(); return 1;
		case EvExit: gs_exit();
	}
	return WsViewer::uievent(e);
}
