
#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <queue>
#include <thread>
#include <string>
#include <Windows.h>
#define __STDC_CONSTANT_MACROS
#define SDL_MAIN_HANDLED
#define MAX_AUDIO_FARME_SIZE 96000
#define NUMBUFFERS (4)
#define SFM_REFRESH_EVENT  (SDL_USEREVENT + 1)//刷新
#define SFM_BREAK_EVENT  (SDL_USEREVENT + 2)//退出

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/samplefmt.h>
#include <libavutil/time.h>
#include <libswresample/swresample.h>
#include <SDL/SDL.h>
#include <SDL/SDL_thread.h>
#include <SDL/SDL_main.h>
#include "al.h"
#include "alc.h"
#include "week6.h"
}

typedef struct _tFrame {//存解码之后的音频
	void* data;
	int size;
	int samplerate;
	double audio_clock;
	int64_t audio_timestamp;
}TFRAME, *PTFRAME;


using namespace std;
#pragma comment(lib ,"SDL2.lib")
#pragma comment(lib ,"SDL2main.lib")

int thread_exit = 0;//退出标识
bool thread_pause = false;//暂停标识
bool seek_req = false;
double increase = 0;
bool fullscreen = false;//窗口最大化标识
bool g_IsMaxWindow = false;
int  g_FastForward = 0;// 0为默认播放速度，1为1.5倍，2为2倍

void ControlSpeed(int& timeInterval, bool& faster, bool& slower)
{
	switch (g_FastForward)
	{
	case 0://原速播放
	{
		if (faster) {
			std::this_thread::sleep_for(std::chrono::milliseconds(timeInterval) / 2);
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(timeInterval));
		}
		if (slower) {
			SDL_Delay(20);
		}
		break;
	}
	case 1://1.5倍速
	{
		if (faster) {
			std::this_thread::sleep_for(std::chrono::milliseconds(int(timeInterval - timeInterval/3)) / 2);
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(int(timeInterval - timeInterval / 3)));
		}
		if (slower) {
			SDL_Delay(20);
		}
		break;
	}
	case 2://2倍速
	{
		if (faster) {
			std::this_thread::sleep_for(std::chrono::milliseconds(int(timeInterval - timeInterval / 2)) / 2);
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(int(timeInterval - timeInterval / 2)));
		}
		if (slower) {
			SDL_Delay(10);
		}
		break;
	}
	default:
		break;
	}
	 


}

int sfp_refresh_thread(int timeInterval, bool& faster, bool& slower) {
	thread_exit = 0;
	thread_pause = 0;

	while (!thread_exit) {
		if (!thread_pause) {
			SDL_Event event;//设置一个事件不断发送
			event.type = SFM_REFRESH_EVENT;//刷新
			SDL_PushEvent(&event);//发送一个事件
		}
		ControlSpeed(timeInterval, faster, slower);
	}
	thread_exit = 0;
	thread_pause = 0;

	SDL_Event event;
	event.type = SFM_BREAK_EVENT;
	SDL_PushEvent(&event);

	return 0;
}
//初始化openal
void initOpenal(ALuint source)
{
	ALfloat SourceP[] = { 0.0, 0.0, 0.0 };
	ALfloat SourceV[] = { 0.0, 0.0, 0.0 };// 源声音的位置
	ALfloat ListenerPos[] = { 0.0, 0, 0 };// 源声音的速度
	ALfloat ListenerVel[] = { 0.0, 0.0, 0.0 };
	ALfloat ListenerOri[] = { 0.0, 0.0, -1.0,  0.0, 1.0, 0.0 };
	alSourcef(source, AL_PITCH, 1.0);
	alSourcef(source, AL_GAIN, 1.0);
	alSourcefv(source, AL_POSITION, SourceP);
	alSourcefv(source, AL_VELOCITY, SourceV);
	alSourcef(source, AL_REFERENCE_DISTANCE, 50.0f);
	alSourcei(source, AL_LOOPING, AL_FALSE);
	alDistanceModel(AL_LINEAR_DISTANCE_CLAMPED);
	alListener3f(AL_POSITION, 0, 0, 0);

}


std::queue<PTFRAME> queueData; //保存解码后数据
ALuint m_source;
double audio_pts;
int64_t audio_timestamp;

int SoundCallback(ALuint & bufferID) {
	if (queueData.empty())//队列为空则退出
		return -1;
	PTFRAME frame = queueData.front();//读取队列中的一帧
	queueData.pop();//该帧数据出队
	if (frame == nullptr)
		return -1;
	alBufferData(bufferID, AL_FORMAT_STEREO16, frame->data, frame->size, frame->samplerate);//把数据写入buffer
	alSourceQueueBuffers(m_source, 1, &bufferID);//将buffer放回缓冲区
	audio_pts = frame->audio_clock;//音频事件

	//释放数据
	if (frame) {
		av_free(frame->data);
		delete frame;
	}
	return 0;
}

//sdl键盘控制
int sfp_control_thread(float& volumn, bool& volumnChange)
{
	const Uint8* state = SDL_GetKeyboardState(NULL);
	bool key_space_down = false;
	bool key_plus_down = false;
	bool key_minus_down = false;
	while (!thread_exit)
	{
		if (state[SDL_SCANCODE_U] && !key_plus_down) {//音量控制
			key_plus_down = true;
			if (volumn < 50) {
				volumn += 0.1;
				cout << "volumn up" << endl;
			}
		}
		else if (!state[SDL_SCANCODE_U] && key_plus_down) {
			key_plus_down = false;
		}

		if (state[SDL_SCANCODE_D] && !key_minus_down) {
			key_minus_down = true;
			if (volumn > 0) {
				volumn -= 0.1;
				cout << "volumn down" << endl;
			}
		}
		else if (!state[SDL_SCANCODE_D] && key_minus_down) {
			key_minus_down = false;
		}
		volumnChange = (key_plus_down || key_minus_down);

		if (state[SDL_SCANCODE_F] && !g_IsMaxWindow)//窗口全屏
		{
			g_IsMaxWindow = true;
			HWND hWnd =  FindWindowA(NULL, "week6");
			SetWindowLong(hWnd, GWL_STYLE, GetWindowLong(hWnd, GWL_STYLE) & ~WS_CAPTION);
			ShowWindow(hWnd, SW_SHOWMAXIMIZED);
			cout << "ful window" << endl;
		}

		if (state[SDL_SCANCODE_A] && (g_FastForward == 0))//倍速控制
		{
			g_FastForward = 1;
		}
		if (state[SDL_SCANCODE_B] && (g_FastForward == 0))
		{
			g_FastForward = 2;
		}
		if (state[SDL_SCANCODE_C])
		{
			g_FastForward = 0;
		}
	
	
		state = SDL_GetKeyboardState(NULL);	//更新键盘状态
	}
	return 0;
}
void forward_func(double second) {//音频前进功能
	double target_pts = audio_pts + second;

	while (!queueData.empty()) {
		PTFRAME frame = queueData.front();
		queueData.pop();
		if (frame == nullptr)
			return;
		if (frame->audio_clock >= target_pts) {
			break;
		}
		if (frame) { 
			av_free(frame->data);
			delete frame;
		}
	}
}
//sdl渲染画面
int sdlplayer(string filePath) {
	AVFormatContext* pFormatCtx;
	int	i, videoindex;
	AVCodecContext* pCodecCtx;
	AVCodec* pCodec;
	AVFrame* pFrame, * pFrameYUV;
	unsigned char* out_buffer;
	AVPacket* packet;
	int ret, got_pic;

	//------------SDL----------------
	int screen_w, screen_h;
	
	SDL_Thread* video_tid;
	SDL_Event event;

	struct SwsContext* img_convert_ctx;
	av_register_all();//注册所有组件
	avformat_network_init();
	pFormatCtx = avformat_alloc_context();

	if (avformat_open_input(&pFormatCtx, filePath.c_str(), NULL, NULL) != 0) {//打开输入视频文件
		cout << "Couldn't open input stream" << endl;
		return -1;
	}
	if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {//获取视频文件信息
		cout << "Couldn't find stream information" << endl;
		return -1;
	}
	videoindex = -1;
	//video
	for (i = 0; i < pFormatCtx->nb_streams; i++)
		if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
			videoindex = i;
			break;
		}
	if (videoindex == -1) {
		cout<<"Didn't find a video stream."<<endl;
		return -1;
	}
	pCodecCtx = pFormatCtx->streams[videoindex]->codec;
	pCodec = avcodec_find_decoder(pCodecCtx->codec_id);//查找解码器
	if (pCodec == NULL) {
		cout<<"Codec not found."<<endl;
		return -1;
	}
	if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {//打开解码器
		cout<<"Could not open codec."<<endl;
		return -1;
	}
	pFrame = av_frame_alloc();//分配空间
	pFrameYUV = av_frame_alloc();

	out_buffer = (unsigned char*)av_malloc(av_image_get_buffer_size(AV_PIX_FMT_YUV420P, pCodecCtx->width, pCodecCtx->height, 1));
	av_image_fill_arrays(pFrameYUV->data, pFrameYUV->linesize, out_buffer,
		AV_PIX_FMT_YUV420P, pCodecCtx->width, pCodecCtx->height, 1);


	av_dump_format(pFormatCtx, 0, filePath.c_str(), 0);

	img_convert_ctx = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt,
		pCodecCtx->width, pCodecCtx->height, AV_PIX_FMT_YUV420P, SWS_BICUBIC, NULL, NULL, NULL);

	//放sdl
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)) {
		cout<<"Could not initialize SDL "<<endl;
		return -1;
	}
	SDL_Window* screen;//窗口
	screen_w = 1080;
	screen_h = 720;
	screen = SDL_CreateWindow("week6", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		screen_w, screen_h, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

	if (!screen) {
		cout << "SDL: could not create window - exiting:%s" << endl;
		return -1;
	}
	SDL_Renderer* sdlRenderer = SDL_CreateRenderer(screen, -1, 0);//渲染器
	SDL_Texture* sdlTexture = SDL_CreateTexture(sdlRenderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING, pCodecCtx->width, pCodecCtx->height);//纹理
	Uint32 pixformat = 0;

	pixformat = SDL_PIXELFORMAT_IYUV;

	SDL_Rect sdlRect;//窗口
	sdlRect.x = 0;
	sdlRect.y = 0;
	sdlRect.w = screen_w;
	sdlRect.h = screen_h;

	packet = (AVPacket*)av_malloc(sizeof(AVPacket));
	double frameRate = (double)pCodecCtx->framerate.num / pCodecCtx->framerate.den;
	bool faster = false;
	bool slower = false;
	std::thread refreshThread(sfp_refresh_thread, (int)(frameRate), std::ref(faster), std::ref(slower));	//视频更新线程
	refreshThread.detach();

	double video_pts = 0;
	double delay = 0;
	while (true) {
	
			while (1) {
				if (av_read_frame(pFormatCtx, packet) < 0)//读取一帧压缩数据
					thread_exit = 1;

				if (packet->stream_index == videoindex)
					break;
			}
			ret = avcodec_send_packet(pCodecCtx, packet);
			got_pic = avcodec_receive_frame(pCodecCtx, pFrame);
			if (ret < 0) {
				cout << "Decode Error." << endl;
				return -1;
			}
			if (!got_pic) {
				sws_scale(img_convert_ctx, (const unsigned char* const*)pFrame->data, pFrame->linesize, 0,
					pCodecCtx->height, pFrameYUV->data, pFrameYUV->linesize);
			}//显示视频画面
			while (1) {
				SDL_WaitEvent(&event);
				if (event.type == SFM_REFRESH_EVENT) {
					if (queueData.empty()) {   //队空情况
						sws_freeContext(img_convert_ctx);
						SDL_Quit();
						av_frame_free(&pFrameYUV);
						av_frame_free(&pFrame);
						avcodec_close(pCodecCtx);
						avformat_close_input(&pFormatCtx);
					}
					if (true) {//比较视音频快慢
						video_pts = (double)pFrame->pts * av_q2d(pFormatCtx->streams[videoindex]->time_base); //获得视频时间戳:pts*时机可得时间
						delay = audio_pts - video_pts;//计算时间差
						if (delay > 0.03) {
							faster = true;
						}
						else if (delay < -0.03) {
							faster = false;
							slower = true;
						}
						else {
							faster = false;
							slower = false;
						}

					}

					SDL_UpdateTexture(sdlTexture, NULL, pFrameYUV->data[0], pFrameYUV->linesize[0]);
					SDL_RenderClear(sdlRenderer);
					SDL_RenderCopy(sdlRenderer, sdlTexture, NULL, NULL);
					SDL_RenderPresent(sdlRenderer);
					av_free_packet(packet);
					break;
				}
				else if (event.type == SDL_KEYDOWN) //前进功能
				{
					switch (event.key.keysym.sym) {
					case SDLK_SPACE:
						thread_pause = thread_pause ? false : true;
						break;
					case SDLK_i:
						increase = 10.0;
						av_seek_frame(pFormatCtx, videoindex, (video_pts + increase + delay * frameRate) / av_q2d(pFormatCtx->streams[videoindex]->time_base), AVSEEK_FLAG_BACKWARD);
						avcodec_flush_buffers(pCodecCtx);
						seek_req = true;
						cout << "+10S" << endl;
						break;
					
					case SDLK_j:
						increase = 30.0;
						av_seek_frame(pFormatCtx, videoindex, (video_pts + increase + delay * frameRate) / av_q2d(pFormatCtx->streams[videoindex]->time_base), AVSEEK_FLAG_BACKWARD);
						avcodec_flush_buffers(pCodecCtx);
						seek_req = true;
						cout << "+30s" << endl;
						break;

					default:
						break;
					}
				}
				else if (event.type == SDL_WINDOWEVENT)
				{
					SDL_GetWindowSize(screen, &screen_w, &screen_h);

				}
				else if (event.type == SDL_QUIT) {
					thread_exit = 1;
					sws_freeContext(img_convert_ctx);
					SDL_Quit();
					av_frame_free(&pFrameYUV);
					av_frame_free(&pFrame);
					avcodec_close(pCodecCtx);
					avformat_close_input(&pFormatCtx);
				}
				else if (event.type == SFM_BREAK_EVENT) {
					break;
				}
			}
			av_packet_unref(packet);

		}


	
	sws_freeContext(img_convert_ctx);
	SDL_Quit();
	av_packet_unref(packet);
	av_frame_free(&pFrameYUV);
	av_frame_free(&pFrame);
	avcodec_close(pCodecCtx);
	avformat_close_input(&pFormatCtx);
}

int main(int agrc, char* argv[]) {

	string filepath = argv[1];  //输入文件mp4
	//ffmepg相关变量
	AVFormatContext* pFormatCtx; ////AVFormatContext主要存储视音频封装格式中包含的信息  
	unsigned  i;
	int audioindex;//音频流所在序号
	AVCodecContext* pCodecCtx_audio;//AVCodecContext，存储该音频流使用解码方式的相关数据
	AVCodec* pCodec_audio;//音频解码器 
	AVFrame* pFrame_audio;

	float volumn = 1.0;  //音量
	bool Vchange = false; //音量是否改变
	bool fast_forward_10 = false;
	bool fast_forward_30 = false;
	SwrContext* swrCtx;
	double audio_clock = 0;
	av_register_all();	//初始化libformat库和注册编解码器
	avformat_network_init();
	avcodec_register_all();
	pFormatCtx = avformat_alloc_context();

	//audio
	if (avformat_open_input(&pFormatCtx, filepath.c_str(), NULL, NULL) != 0) {
		cout<<"Couldn't open input stream."<<endl;
		return -1;
	}
	//获取流信息
	if (avformat_find_stream_info(pFormatCtx, NULL) < 0) {
		cout<<"Couldn't find stream information."<<endl;
		return -1;
	}
	//获取各个媒体流的编码器信息，找到对应的type所在的pFormatCtx->streams的索引位置，初始化编码器。播放音频时type是AUDIO
	audioindex = -1;
	//找到音频流的序号
	for (int i = 0; i < pFormatCtx->nb_streams; i++)
		if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
			audioindex = i;
			break;
		}
	if (audioindex == -1) {
		cout<<"Didn't find a video stream."<<endl;
		return -1;
	}
	//获取解码器
	pCodec_audio = avcodec_find_decoder(pFormatCtx->streams[audioindex]->codecpar->codec_id);
	if (pCodec_audio == NULL) {
		cout<<"Codec not found."<<endl;
		return -1;
	}
	pCodecCtx_audio = avcodec_alloc_context3(pCodec_audio);
	avcodec_parameters_to_context(pCodecCtx_audio, pFormatCtx->streams[audioindex]->codecpar);
	pCodecCtx_audio->pkt_timebase = pFormatCtx->streams[audioindex]->time_base;
	//打开解码器
	if (avcodec_open2(pCodecCtx_audio, pCodec_audio, NULL) < 0) {
		cout<<"Couldn't open codec."<<endl;
		return -1;
	}

	//内存分配
	AVPacket* packet;//解码前的包
	packet = (AVPacket*)av_malloc(sizeof(AVPacket));
	pFrame_audio = av_frame_alloc();


	//设置输出的音频参数
	int out_nb_samples = 1024;//单个通道样本个数
	int out_sample_rate = 44100;//输出时采样率,CD一般为44100HZ
	int in_sample_rate = pCodecCtx_audio->sample_rate; //输入采样率
	enum AVSampleFormat in_sample_fmt = pCodecCtx_audio->sample_fmt;  //输入的采样格式  
	enum AVSampleFormat out_sample_fmt = AV_SAMPLE_FMT_S16; //输出采样格式16bit PCM  
	uint64_t in_ch_layout = pCodecCtx_audio->channel_layout; //输入的声道布局   
	uint64_t out_ch_layout = AV_CH_LAYOUT_STEREO; //输出的声道布局（立体声）
	int out_channels = av_get_channel_layout_nb_channels(out_ch_layout);//根据通道布局类型获取通道数

	int out_buffer_size = av_samples_get_buffer_size(NULL, out_channels, out_nb_samples, out_sample_fmt, 1);//根据通道数、样本个数、采样格式分配内存
	uint8_t* out_buffer_audio;


	pCodecCtx_audio->channel_layout = av_get_default_channel_layout(pCodecCtx_audio->channels);


	//swr
	swrCtx = swr_alloc();
	swrCtx = swr_alloc_set_opts(swrCtx,
		out_ch_layout, out_sample_fmt, out_sample_rate,
		in_ch_layout, in_sample_fmt, in_sample_rate,
		0, NULL); //设置参数
	swr_init(swrCtx); //初始化


	
	int ret;
	while (av_read_frame(pFormatCtx, packet) >= 0) {//读取下一帧数据
		if (packet->stream_index == audioindex) {
			ret = avcodec_send_packet(pCodecCtx_audio, packet);
			if (ret < 0) {
				cout << "avcodec_send_packet：" << ret << endl;
				continue;
			}
			while (ret >= 0) {
				ret = avcodec_receive_frame(pCodecCtx_audio, pFrame_audio);
				if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
					break;
				}
				else if (ret < 0) {
					cout << "avcodec_receive_frame：" << AVERROR(ret) << endl;
					return -1;
				}

				if (ret >= 0) {
					out_buffer_audio = (uint8_t*)av_malloc(MAX_AUDIO_FARME_SIZE * 2);//*2是保证输出缓存大于输入数据大小
					swr_convert(swrCtx, &out_buffer_audio, MAX_AUDIO_FARME_SIZE, (const uint8_t * *)pFrame_audio->data, pFrame_audio->nb_samples);//重采样
					out_buffer_size = av_samples_get_buffer_size(NULL, out_channels, pFrame_audio->nb_samples, out_sample_fmt, 1);
					PTFRAME frame = new TFRAME;
					frame->data = out_buffer_audio;
					frame->size = out_buffer_size;
					frame->samplerate = out_sample_rate;
					audio_clock = av_q2d(pCodecCtx_audio->time_base) * pFrame_audio->pts;
					frame->audio_clock = audio_clock;

					queueData.push(frame);  //解码后数据存入队列
				}
			}
		}
		av_packet_unref(packet);
	}


	ALCdevice* pDevice;
	ALCcontext* pContext;

	pDevice = alcOpenDevice(NULL);
	pContext = alcCreateContext(pDevice, NULL);
	alcMakeContextCurrent(pContext);

	if (alcGetError(pDevice) != ALC_NO_ERROR)
		return AL_FALSE;

	ALuint m_buffers[NUMBUFFERS];
	alGenSources(1, &m_source);
	if (alGetError() != AL_NO_ERROR) {
		cout << "Error generating audio source." << endl;
		return -1;
	}
	initOpenal(m_source);

	alGenBuffers(NUMBUFFERS, m_buffers); //创建缓冲区
	ALint iQueuedBuffers;
	ALint processed1 = 0;
	alGetSourcei(m_source, AL_BUFFERS_PROCESSED, &processed1);
	std::thread controlopenal{ sfp_control_thread, std::ref(volumn), std::ref(Vchange) };
	controlopenal.detach();
	std::thread sdlplay{ sdlplayer, filepath };
	sdlplay.detach();
	ALint processed;
	ALint  state;
	for (int i = 0; i < NUMBUFFERS; i++) {//填充数据
		SoundCallback(m_buffers[i]);
	}
	alSourcePlay(m_source);//播放音频
	while (!queueData.empty()) {  //队列为空后停止播放
		if (seek_req) {
			forward_func(increase);
			seek_req = false;
			increase = 0;
		}
		 processed = 0;
		alGetSourcei(m_source, AL_BUFFERS_PROCESSED, &processed);
		while (processed > 0) {
			ALuint bufferID = 0;
			alSourceUnqueueBuffers(m_source, 1, &bufferID);
			SoundCallback(bufferID);
			processed--;
		}
		if (Vchange)
		{
			alSourcef(m_source, AL_GAIN, volumn);
		}
		
		alGetSourcei(m_source, AL_SOURCE_STATE, &state);
		if (thread_pause)
		{
			alSourcePause(m_source);
		}
		else if (state != AL_PLAYING) {
			alGetSourcei(m_source, AL_BUFFERS_QUEUED, &iQueuedBuffers);
			if (iQueuedBuffers) {
				alSourcePlay(m_source);
			}
			else {
				break;
			}
		}
		 if (thread_exit) {
			 alSourceStop(m_source);
			 alSourcei(m_source, AL_BUFFER, 0);
			 alDeleteBuffers(NUMBUFFERS, m_buffers);
			 alDeleteSources(1, &m_source);
			 break;
		 }

	}

	alSourceStop(m_source);
	alSourcei(m_source, AL_BUFFER, 0);
	alDeleteBuffers(NUMBUFFERS, m_buffers);
	alDeleteSources(1, &m_source);



	av_frame_free(&pFrame_audio);
	swr_free(&swrCtx);


	ALCcontext* pCurContext = alcGetCurrentContext();
	ALCdevice* pCurDevice = alcGetContextsDevice(pCurContext);

	alcMakeContextCurrent(NULL);
	alcDestroyContext(pCurContext);
	alcCloseDevice(pCurDevice);


	return 0;
}



// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门提示: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
