#include "myarea.h"
#include <gtkmm/application.h>
#include <gtkmm/window.h>
#include <iostream>
#include <sys/shm.h>
#include <unistd.h>

using namespace std;

#define SHM_SIZE 1024
float *MakeFloatSharedMemory2(int HowBig2);



// compile g++ -std=c++11 main.cpp myarea.cpp -o test `pkg-config --cflags --libs gtkmm-3.0`
//int const count=0;
int main(int argc, char** argv)
{

    //int count=0;
    double x=0;
    double y=0;
    Glib::RefPtr<Gtk::Application>  app = Gtk::Application::create(argc, argv, "org.gtkmm.example");

    Gtk::Window win;
    win.set_title("DrawingArea");
    win.set_default_size(1920,1080);
    win.set_position(Gtk::WIN_POS_CENTER);


    for(;;){
        float *data2; //pointer to shared memory
        data2=MakeFloatSharedMemory2(2);
        while(x==0){
            x=data2[1];
        }
        while(y==0){
            y=data2[0];
          }

        //   std::cout << count << std::endl;
        //       if ((960+0.1*2743+50 > 960+(0.36-x)*2743) && (960+0.1*2743-50 < 960+(0.36-x)*2743))
        //   {
        // count=count+1;
        //
        // if (count==1000)
        // {
        //   system("amarok beep.wav");
        //   count=0;
        // }
        //   }



       MyArea area;
       //std::cout << xc << std::endl;
       win.add(area);
       area.show();
       std::cout << "before"<< std::endl;
       return app->run(win);
       std::cout << "after"<< std::endl;
       return 0;
    }

}

//    /////////////////////////////SHARED MEMORY -DISPLAY///////////////////////////////// 8>7

float *MakeFloatSharedMemory2(int HowBig2)
{
    key_t key2;
    int shmid2;
    float *dataShared2;

    dataShared2 = (float *) malloc(HowBig2*sizeof(float));
    /* make the key */
    if ( (key2=ftok("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile2",'R')) == -1 )
        {
        perror("ftok-->");
        exit(1);
        }

    if ((shmid2 = shmget(key2, SHM_SIZE, 0666 | IPC_CREAT)) == -1)
        {
        perror("shmget");
        exit(1);
        }

    dataShared2 = (float *) shmat(shmid2, (void *)0, 0);

    if (dataShared2 == (float *) (-1))
        {
        perror("shmat");
        exit(1);
        }

    for(int i=0;i<HowBig2;i++)
        {
        dataShared2[i]=0.0;
        }

return dataShared2;
}
