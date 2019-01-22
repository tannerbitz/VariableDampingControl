#include "myarea.h"
#include <cairomm/context.h>
#include <glibmm/main.h>
#include <cmath>
#include <iostream>
#include <sys/shm.h>
#include <unistd.h>


using namespace std;

#define SHM_SIZE 1024

float *MakeFloatSharedMemory2(int HowBig2);



MyArea::MyArea()
{

  Glib::signal_timeout().connect( sigc::mem_fun(*this, &MyArea::on_timeout), 1 );
}

MyArea::~MyArea()
{
}

bool MyArea::on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
  // This is where we draw on the window
  Gtk::Allocation allocation = get_allocation();
  const int width = allocation.get_width();
  const int height = allocation.get_height();
  const int lesser = MIN(width, height);
  int space=600;
  float *data2; //pointer to shared memory
  data2=MakeFloatSharedMemory2(2);

  //data2=MakeFloatSharedMemory2(2);

  // coordinates for the center of the window
  int xc, yc;
  xc = width / 2;
  yc = height / 2;
  double x=0;
  double y=0;
  int count=0;


  cr->set_line_width(lesser * 0.01);  // outline thickness changes
                                      // with window size

  // first draw a simple unclosed arc
  /*cr->save();
  cr->arc(width / 3.0, height / 4.0, lesser / 4.0, -(M_PI / 5.0), M_PI);
  cr->close_path();   // line back to start point
  cr->set_source_rgb(0.0, 0.8, 0.0);
  cr->fill_preserve();
  cr->restore();  // back to opaque black
  cr->stroke();   // outline it*/

  // now draw a circle
  cr->save();
  cr->arc(xc+0.1*2743, yc, lesser / 20.0, 0.0, 2.0 * M_PI); // full circle
  cr->set_source_rgba(0.0, 0.0, 0.8, 0.6);    // partially translucent
  cr->fill_preserve();
  cr->restore();  // back to opaque black
  cr->stroke();

  cr->save();
  cr->arc(xc-0.1*2743, yc, lesser / 20.0, 0.0, 2.0 * M_PI); // full circle
  cr->set_source_rgba(0.0, 0.0, 0.8, 0.6);    // partially translucent
  cr->fill_preserve();
  cr->restore();  // back to opaque black
  cr->stroke();


  cr->save();

  while(x==0){
    x=data2[1];
  }
  while(y==0){
    y=data2[0];
  }
  //std::cout << y << std::endl;
  //std::cout << "y" << std::endl;
 // std::cout << yc << std::endl;
  cr->arc(xc+(0.36-x)*2743, yc+(-0.4-y)*1280, lesser / 30.0, 0.0, 2.0 * M_PI); // full circle
  cr->set_source_rgba(0.0, 0.8, 0.0, 1.0);    // partially translucent
  cr->fill_preserve();
  cr->restore();  // back to opaque black
  cr->stroke();


  cr->save();
  cr->move_to(xc-.165*2743, 200);
  cr->line_to(xc-.165*2743, 800);
  cr->restore();  // back to opaque black
  cr->stroke();


  cr->save();
  cr->move_to(xc+.165*2743, 200);
  cr->line_to(xc+.165*2743, 800);
  cr->restore();  // back to opaque black
  cr->stroke();
  //std::cout << count << std::endl;
  // cout << '\a';
  //std::cout << x << std::endl;
  usleep(1000);






  if ((xc+0.1*2743+1 > xc+(0.36-x)*2743) && (xc+0.1*2743-1 < xc+(0.36-x)*2743) && (yc+5 > yc+(-0.4-y)*1280) && (yc-5 < yc+(-0.4-y)*1280))
  {
    system("amarok beep.wav");
  }

  //Beep(587,500);
  /*// and finally an ellipse
  double ex, ey, ew, eh;
  // center of ellipse
  ex = xc;
  ey = 3.0 * height / 4.0;
  // ellipse dimensions
  ew = 3.0 * width / 4.0;
  eh = height / 3.0;

  cr->save();

  cr->translate(ex, ey);  // make (ex, ey) == (0, 0)
  cr->scale(ew / 2.0, eh / 2.0);  // for width: ew / 2.0 == 1.0
                                  // for height: eh / 2.0 == 1.0

  cr->arc(0.0, 0.0, 1.0, 0.0, 2 * M_PI);  // 'circle' centered at (0, 0)
                                          // with 'radius' of 1.0

  cr->set_source_rgba(0.8, 0.0, 0.0, 0.7);
  cr->fill_preserve();
  cr->restore();  // back to opaque black
  cr->stroke();*/

  return true;
}

bool MyArea::on_timeout()
{
    // force our program to redraw the entire clock.
    auto win = get_window();
    if (win)
    {
        Gdk::Rectangle r(0, 0, get_allocation().get_width(),
                get_allocation().get_height());
        win->invalidate_rect(r, false);
    }
    return true;
}





// //    /////////////////////////////SHARED MEMORY -DISPLAY///////////////////////////////// 8>7
//
// float *MakeFloatSharedMemory2(int HowBig2)
// {
//     key_t key2;
//     int shmid2;
//     float *dataShared2;
//
//     dataShared2 = (float *) malloc(HowBig2*sizeof(float));
//     /* make the key */
//     if ( (key2=ftok("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile2",'R')) == -1 )
//         {
//         perror("ftok-->");
//         exit(1);
//         }
//
//     if ((shmid2 = shmget(key2, SHM_SIZE, 0666 | IPC_CREAT)) == -1)
//         {
//         perror("shmget");
//         exit(1);
//         }
//
//     dataShared2 = (float *) shmat(shmid2, (void *)0, 0);
//
//     if (dataShared2 == (float *) (-1))
//         {
//         perror("shmat");ExampleApplication::create();
//         exit(1);
//         }
//
//     for(int i=0;i<HowBig2;i++)
//         {
//         dataShared2[i]=0.0;
//         }
//
// return dataShared2;
// }
