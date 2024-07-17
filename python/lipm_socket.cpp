#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <time.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <math.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/ipc.h>

#define PORT 7072
#define PORT1 7071

using namespace std;

int sock = 0, sock1, valread;
int new_socket;
struct sockaddr_in serv_addr, serv_addr1;
const char* hello = "Kello from client";
bool mpc_start_init_bool1 = false;
bool mpc_start_init_bool = false;
bool mpc_start_init_bool2 = false;

double buffer[50] = {2, 3, 4, 5, 6, 
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 1.0, 2, 
    3, 4, 5, 6, 0, 0,
    0, 99, 100
};//send

double buffer1[51] = {3000000, 3000000, 3000000, 3000000, 3000000, 3000000, 
3000000, 3000000, 3000000, 3000000, 3000000, 3000000,
3000000, 3000000, 3000000, 3000000, 3000000, 3000000,
3000000, 3000000, 3000000, 3000000, 3000000, 3000000,
3000000, 3000000, 3000000, 3000000, 3000000, 3000000,
3000000, 3000000, 3000000, 3000000, 3000000, 3000000,
3000000, 3000000, 3000000, 3000000, 3000000, 3000000, 
3000000, 3000000, 3000000, 3000000, 3000000, 3000000, 
3000000, 3000000, 3000000
};//receive

class CSharedMemory
{
 
private :
    

 
 
public :
 
    void setShmId(int key);
    int getShmId();
    void setKey(key_t key);
 
    void setupSharedMemory(int size);
    void setupSharedMemoryRead(int size);
    void attachSharedMemory();
    void attachSharedMemoryint();
    void close();
    int m_shmid;   
    key_t m_key;
    double *m_shared_memory;
    int *m_shared_memory_int;
    
};
 
 
void CSharedMemory::setShmId( int id )
{
    m_shmid = id;
}
 
 
void CSharedMemory::setKey( key_t key )
{
    m_key = key;
}
 
 
void CSharedMemory::setupSharedMemory(int size)
{
   // Setup shared memory, 11 is the size
   if ((m_shmid = shmget(m_key, size , IPC_CREAT | 0666)) < 0)
   {
      printf("Error getting shared memory id");
      exit( 1 );
   }
}

void CSharedMemory::setupSharedMemoryRead(int size)
{
    m_shmid = shmget(m_key, size , IPC_CREAT | 0666);
   // Setup shared memory, 11 is the size
   if ((m_shmid = shmget(m_key, size , 0666|IPC_EXCL)) < 0)
   {
      printf("Error getting shared memory id");
      exit( 1 );
   }
}
 
void CSharedMemory::attachSharedMemory()
{
   // Attached shared memory
   m_shared_memory = (double*)shmat(m_shmid,NULL,0);
   if ((*m_shared_memory) == -1)
   {
      printf("Error attaching shared memory id");
      exit(1);
   }
}

void CSharedMemory::attachSharedMemoryint()
{
   // Attached shared memory
   m_shared_memory_int = (int*)shmat(m_shmid,NULL,0);
   if ((*m_shared_memory_int) == -1)
   {
      printf("Error attaching shared memoryint id");
      exit(1);
   }
}
 
void CSharedMemory::close()
{
   // Detach and remove shared memory
   shmctl(m_shmid,IPC_RMID,NULL);
 
}

CSharedMemory mpc_start_init, state_init, statemachine, desired_val;

void proc_recv(){

    while(true)
    {
        buffer[0] = statemachine.m_shared_memory_int[0];
        if(buffer[0] == 2 && mpc_start_init_bool == true && mpc_start_init_bool2 == false)
        {
            mpc_start_init.m_shared_memory_int[0] = 2;
        
        }   

        if(buffer[0] == 1 && mpc_start_init_bool == false)
        { 
            std::cout << "send oK "<< buffer[0] << std::endl;
            send(sock1, buffer, sizeof(buffer), 0);
            mpc_start_init_bool = true;
            mpc_start_init_bool1 = false;
            //std::cout << "KK2 "<< statemachine.m_shared_memory_int[0] << " " << mpc_start_init.m_shared_memory_int[0]<< " " << mpc_start_init_bool<< " " << mpc_start_init_bool1<< " " << mpc_start_init_bool2<<std::endl;
        
        }
        else if(buffer[0] == 2 && mpc_start_init_bool1 == false)// && buffer1[0] == 2)
        {
            send(sock1, buffer, sizeof(buffer), 0);
            mpc_start_init_bool1 = true;
            mpc_start_init_bool2 = false;
            mpc_start_init_bool = false;
            //std::cout <<"FF2 " << statemachine.m_shared_memory_int[0] << " " << mpc_start_init.m_shared_memory_int[0]<< " " << mpc_start_init_bool<< " " << mpc_start_init_bool1<< " " << mpc_start_init_bool2<<std::endl;
        
        }
        else if(buffer[0] == 3)// && buffer1[0] == 2)
        {
            send(sock1, buffer, sizeof(buffer), 0);
            mpc_start_init_bool1 = true;
            mpc_start_init_bool = false;
            //std::cout <<"Finish"<< std::endl;
            break;
        }
        /*else if(buffer[0] == 1)
        {
            std::cout << "SEND NON OK " << mpc_start_init_bool << " " << buffer[0] <<  std::endl;
        }*/

        std::copy(&desired_val.m_shared_memory[0], &desired_val.m_shared_memory[0] + 49, &buffer[1]);
    }

}

void proc_recv1(){

    while(true)
    {
        auto startTime1 = std::chrono::system_clock::now();
        valread = recv(new_socket, buffer1, sizeof(buffer1), MSG_WAITALL);
        auto startTime2 = std::chrono::system_clock::now();
        auto elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(startTime2 - startTime1).count();
      
        if(valread > 0)
                std::cout << "val read" << valread << std::endl;
            
        if(valread == sizeof(buffer1))// && elapsed2 > 1)
        {
            
            std::copy(&buffer1[1], &buffer1[1] + 50, &state_init.m_shared_memory[0]);

            mpc_start_init.m_shared_memory_int[0] = buffer1[0];
            
            //if(buffer1[0] == 1)
            //    std::cout << "send OK " << buffer1[0]<< " " << mpc_start_init.m_shared_memory_int[0] << " " << state_init.m_shared_memory[0] <<std::endl;

            if(mpc_start_init.m_shared_memory_int[0] == 1)
            {
                //std::cout << "KKKK1" << std::endl;
                mpc_start_init_bool2 = true;
            }
        }
    }
}

int main(int argc, char const *argv[]) {
    // Create socket file descriptor
    /*if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }

    if ((sock1 = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }*/
    if ((sock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }

    if ((sock1 = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket creation error" << std::endl;
        return -1;
    }

    serv_addr.sin_family = PF_INET;
    serv_addr.sin_port = htons(8076);//8080

    serv_addr1.sin_family = PF_INET;
    serv_addr1.sin_port = htons(7073);//8081

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(PF_INET, "10.112.1.20", &serv_addr.sin_addr)<=0)
    {
        std::cout << "Invalid address/ Address not supported" << std::endl;
        return -1;
    }

    if(inet_pton(PF_INET, "10.112.1.10", &serv_addr1.sin_addr)<=0) 
    {
        std::cout << "Invalid address/ Address not supported" << std::endl;
        return -1;
    }

       // Connect to the server //8080
    while (bind(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cout << "Connection1 ...." << std::endl;
    } 

    // Connect to the server
    while (connect(sock1, (struct sockaddr *)&serv_addr1, sizeof(serv_addr1)) < 0) {
        std::cout << "Connection2 ...." << std::endl;
        //break;
    } //8081

    if (listen(sock, 3) < 0) {
        std::cout << "listen err" << std::endl;
    } //8080

    socklen_t addrlen = sizeof(serv_addr);
    if ((new_socket = accept(sock, (struct sockaddr*)&serv_addr, &addrlen))
        < 0) {
        std::cout << "accept err" <<  strerror(errno) << std::endl;
    } //8080
    
    struct timespec specific_time;
    struct timespec specific_time1;
    struct tm *now;
    int millsec;
    std::chrono::system_clock::time_point startTime;
    std::chrono::system_clock::time_point endTime;
    // Send message to the server
    int maxiter = 0;
    int K = 0;

    int shm_id_;
    if ((shm_id_ = shmget(key_t(1), sizeof(sizeof(int) * 3), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm1 mtx failed " << std::endl;
    }

    if(shmctl(shm_id_, IPC_RMID, NULL) == -1)
    {
        std::cout << "Error Deleting SHAREDMEMORY1"<<std::endl;
    }

    if ((shm_id_ = shmget(key_t(2), sizeof(sizeof(double) * 49), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm2 mtx failed " << std::endl;
    }

    if(shmctl(shm_id_, IPC_RMID, NULL) == -1)
    {
        std::cout << "Error Deleting SHAREDMEMORY2"<<std::endl;
    }

    if ((shm_id_ = shmget(key_t(3), sizeof(sizeof(int) * 3), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm3 mtx failed " << std::endl;
    }

    if(shmctl(shm_id_, IPC_RMID, NULL) == -1)
    {
        std::cout << "Error Deleting SHAREDMEMORY3"<<std::endl;
    }

    if ((shm_id_ = shmget(key_t(4), sizeof(sizeof(double) * 49), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm3 mtx failed " << std::endl;
    }
    if(shmctl(shm_id_, IPC_RMID, NULL) == -1)
    {
        std::cout << "Error Deleting SHAREDMEMORY4"<<std::endl;
    }

    mpc_start_init.setKey(1); //receive
    mpc_start_init.setupSharedMemory(sizeof(int) * 3); //1
    mpc_start_init.attachSharedMemoryint();
    std::cout << "SHAREDMEMORY FIRST OK" << std::endl;
    state_init.setKey(2); //receive
    state_init.setupSharedMemory(sizeof(double) * 50);
    state_init.attachSharedMemory();
    std::cout << "SHAREDMEMORY Second OK" << std::endl;
    statemachine.setKey(3); //send
    statemachine.setupSharedMemoryRead(sizeof(int) * 3);
    statemachine.attachSharedMemoryint();
    std::cout << "SHAREDMEMORY Th2ird  OK"<< std::endl;
    desired_val.setKey(4); //send
    desired_val.setupSharedMemoryRead(sizeof(double) * 49);
    desired_val.attachSharedMemory();

    std::cout << "SHAREDMEMORY Fourth OK" << std::endl;
    
    statemachine.m_shared_memory_int[0] = 90;

    thread proc1(proc_recv);
    thread proc2(proc_recv1);

    while(1)
    {   
        //sleep(0.01)
    }
    return 0;
}
