/*

author:Sean Jiang,Peter Jin.


*/
//#include "stdafx.h"
#include "simulation/structures/queue_struct.hpp"


namespace prx
{
    namespace packages
    {
        namespace crowd
        {

			// int segment_t::global_segment_id = 1e3;
			// int segment_struct_t::global_segment_struct_id = 1e3;
        	
			points_t::points_t()
			{
				
			}
			points_t::points_t(double a,double b,double c)
			{
				x=a;y=b;z=c;
			}

			void points_t::set(double a,double b,double c)
			{
				x=a;y=b;z=c;
			}

			void points_t::set(const std::vector<double>& pt)
			{
				PRX_ASSERT(pt.size()==3);
				x=pt[0];
				y=pt[1];
				z=pt[2];
			}

			bool points_t::equal(points_t p)
			{
				if(this->x==p.x && this->y==p.y)
					return true;
				else
					return false;
			}

			segment_t::segment_t()
		  	{
		  		object_id = -1;
		  	}	


		  	// The new function
			segment_t::segment_t(int object_id, points_t start_point, points_t end_point)
			{
				this->object_id = object_id;
				set(start_point, end_point);
			}

			double segment_t::a()
			{
				double aa = endpt.y - startpt.y;
				if(abs(aa)<1e-6)
					aa=0;
				return aa;
			}

			double segment_t::b()
			{
				double bb = startpt.x - endpt.x;
				if(abs(bb)<1e-6)
					bb=0;
				return bb;
			}

			double segment_t::c()
			{
				double cc = endpt.x*startpt.y - startpt.x*endpt.y;
				if(abs(cc)<1e-6)
					cc=0;
				return cc;
			}

			double segment_t::k()
			{
				double kk;
				if (abs(startpt.x - endpt.x)>1e-10)
					kk = (startpt.y - endpt.y) / (startpt.x - endpt.x);
				else
					kk = (numeric_limits<double>::max)();

				if(abs(kk)<1e-6)
					kk=0;

				return kk;
			}
			double segment_t::p()//y=kx+p
			{
				double pp;
				double pt1x=startpt.x;
				double pt1y=startpt.y;
				double pt2x=endpt.x;
				double pt2y=endpt.y;
				if (abs(pt1x - pt2x)>1e-6 && abs(pt1y-pt2y)>1e-6)
					pp = (pt2y*pt1x - pt2x*pt1y) / (pt1x - pt2x);
				else if(abs(pt1x - pt2x)<1e-6)
					pp = pt1x;
				else if (abs(pt1y - pt2y)<1e-6)
					pp=pt1y;
				else
					pp=(numeric_limits<double>::max)();

				if(abs(pp)<1e-6)
					pp=0;

				return pp;
			}
			vector<double> segment_t::nor(points_t nor_pt)
			{
				double nor_a,nor_b,nor_c;
				vector<double> ret;
				ret.resize(3);
				this->cal_para();
				nor_a=-parab;
				nor_b=paraa;
				if (nor_a!=0 && nor_b!=0)
				{
					nor_c=-nor_a*nor_pt.x-nor_b*nor_pt.y;
				}
				else if(nor_a==0 && nor_b!=0)
				{
					nor_c=-nor_b*nor_pt.y;
				}
				else
				{
					nor_c=-nor_a*nor_pt.x;
				}
				ret[0]=nor_a;
				ret[1]=nor_b;
				ret[2]=nor_c;
				return ret;
			}

			void segment_t::cal_para()
			{
				paraa=a();
				parab=b();
				parac=c();
				parak=k();
				parap=p();
			}

			void segment_t::set(points_t p1,points_t p2)
			{
				startpt=p1;endpt=p2;
				this->cal_para();
			}


			bool segment_t::operator== (const segment_t& seg) const
			{
				return object_id == seg. object_id;
			}



			int segment_t::get_hash_key()
			{
				return seg_key;
			}


			double segment_t::get_length()
			{
				return sqrt((startpt.x-endpt.x)*(startpt.x-endpt.x)+(startpt.y-endpt.y)*(startpt.y-endpt.y));
			}


			vector<double> cart2pol(double x,double y,double z)
			{
				double theta;
				vector<double> pol;
				pol.resize(3,0);
				pol[2]=z;
				pol[1]=sqrt(x*x+y*y);
				if(x==0)
				{
					if(y>0)
						theta=0.5*PI;
					else if(y<0)
						theta=1.5*PI;
					else
					{
						theta=0;
						pol[1]=0;
					}
				}
				else if(y==0)
				{
					if(x>0)
						theta=0;
					else if (x<0)
					{
						theta=PI;

					}
					else
					{
						theta=0;
						pol[1]=0;
					}
				}
					
				else
				{
					if(x>0 && y>0)
						theta=atan(y/x);
					
					else if(x>0 && y<0)
						theta=2*PI-atan(abs(y/x));
					else if (x<0 && y>0)
						theta=PI-atan(abs(y/x));
					else if (x<0 && y<0)
						theta=PI+atan(abs(y/x));
					else
						theta=0;
				}
				pol[0]=theta;
				return pol;
			}

			vector<double> pol2cart(double theta,double r)
			{
				double x,y;
				vector<double> cart;
				cart.resize(2);
				y=sin(theta)*r;
				x=cos(theta)*r;
				if(abs(x)<1e-6)
					x=0;
				if(abs(y)<1e-6)
					y=0;
				cart[0]=x;
				cart[1]=y;
				
				return cart;
			}

			double pt_pt_dist(points_t pt1,points_t pt2)
			{
				double dist=sqrt((pt1.x-pt2.x)*(pt1.x-pt2.x)+(pt1.y-pt2.y)*(pt1.y-pt2.y));
				return dist;
			}

			vector<double> regressline(vector<points_t> pts)
			{
				vector<double> para;
				points_t p1,p2;
				double k,bl;
				double k2,bl2;
				para.resize(3,-1);
				int pts_num=pts.size();
				k=0;bl=0;

				double sig_x=0;
				double sig_y=0;
				double sig_xy=0;
				double sig_x2=0;
				for (int i=0;i<pts_num;i++)
				{
					sig_x=sig_x+pts[i].x;
					sig_y=sig_y+pts[i].y;
					sig_xy=sig_xy+pts[i].x*pts[i].y;
					sig_x2=sig_x2+pts[i].x*pts[i].x;
				}
				double n=pts_num;
				k=(n*sig_xy-sig_x*sig_y)/(n*sig_x2-sig_x*sig_x);
				bl=sig_y/n-k*sig_x/n;
				para[0]=k;
				para[1]=bl;
				return para;
			}

			
			double pt_seg_dist(points_t pt,segment_t* seg) 
			{
				double ret;
				seg->cal_para();

				double seg_k=seg-> parak;
				double nor_k,nor_p;

				double pt_line_intersec_y;
				double pt_line_intersec_x;

				if(seg-> parab==0 || abs(seg-> parab)<1e-6)
				{
					nor_k=0;
					nor_p=pt.y;
					pt_line_intersec_y=pt.y;
					pt_line_intersec_x=-seg-> parac/seg-> paraa;
				}
				else if(abs(seg_k)>1e-6 && seg_k!=0)
				{
					nor_k=-1/seg_k;
					nor_p=pt.y-nor_k*pt.x;
					pt_line_intersec_x=(nor_p-seg-> parap)/(seg-> parak-nor_k);
					pt_line_intersec_y=nor_k*pt_line_intersec_x+nor_p;
				}
				else
				{
					nor_k=99999;
					nor_p=pt.x;
					pt_line_intersec_x=pt.x;
					pt_line_intersec_y=-seg-> parac/seg-> parab;

				}
	
				ret=sqrt((pt.x-pt_line_intersec_x)*(pt.x-pt_line_intersec_x)+(pt.y-pt_line_intersec_y)*(pt.y-pt_line_intersec_y));
				return ret;
			}


			points_t pt_seg_intersec_pt(points_t pt,segment_t* seg)
			{
				points_t ret;
				seg->cal_para();

				double seg_k=seg-> parak;
				double nor_k,nor_p;

				double pt_line_intersec_y;
				double pt_line_intersec_x;

				if(seg-> parab==0 || abs(seg-> parab)<1e-6)
				{
					nor_k=0;
					nor_p=pt.y;
					pt_line_intersec_y=pt.y;
					pt_line_intersec_x=-seg-> parac/seg-> paraa;
				}
				else if(abs(seg_k)>1e-6 && seg_k!=0)
				{
					nor_k=-1/seg_k;
					nor_p=pt.y-nor_k*pt.x;
					pt_line_intersec_x=(nor_p-seg-> parap)/(seg-> parak-nor_k);
					pt_line_intersec_y=nor_k*pt_line_intersec_x+nor_p;
				}
				else
				{
					nor_k=99999;
					nor_p=pt.x;
					pt_line_intersec_x=pt.x;
					pt_line_intersec_y=-seg-> parac/seg-> parab;

				}


				ret.set(pt_line_intersec_x,pt_line_intersec_y,pt.z);
				return ret;
			}


			int judge_bt_points(points_t jbpx, points_t jbp1, points_t jbp2)  //1 btween,0 not between //to judge if the pt x is between pt1 and pt2. 
			{
				if ((jbp1.x - jbpx.x)*(jbpx.x - jbp2.x)>0 || abs((jbp1.x - jbpx.x)*(jbpx.x - jbp2.x)) <1e-6)
				{
					if ((jbp1.y - jbpx.y)*(jbpx.y - jbp2.y)>0 || abs((jbp1.y - jbpx.y)*(jbpx.y - jbp2.y))<1e-6)
					{
						segment_t seg;
						seg. startpt=jbp1;seg. endpt=jbp2;
						if(abs(seg. k()*jbpx.x+seg. p()-jbpx.y)<1e-6)
						{
							return 2; //in the range, and on the segment
						}
						else
						{
							return 1; //in the range, but not on segment
						}
						
					}
					else
					{
						
						return 0; //not in the rectangle range
					}
				}
				else
				{
					
					return 0;
				}

			}

			_numnpt check_seg_intersec(segment_t* s1,segment_t* s2)  //0-not inter 1-inter 2-overlap
			{
				_numnpt ret;
				s1->cal_para();s2->cal_para();

				if(s1->parab!=0 && s2->parab!=0)
				{
					if (abs(s1->parak-s2->parak)<1e-6 || s1->parak-s2->parak==0)
					{
						if(s1->parap==s2->parap || abs(s1->parap-s2->parap)<1e-6)
						{
							ret.vals=2; //overlap
							if (pt_pt_dist(s1->startpt,s2->startpt)<pt_pt_dist(s1->startpt,s2->endpt))
								ret.pt=s2->startpt;
							else
								ret.pt=s2->endpt;
						}
						else
						{
							ret.vals=0;
						}
					
					}
					else
					{		
						ret.pt.x=(s2->parap-s1->parap)/(s1->parak-s2->parak);
						ret.pt.y=s1->parak*ret.pt.x+s1->parap;
						ret.pt.z=s1->endpt.z;
						if(judge_bt_points(ret.pt,s1->startpt,s1->endpt)==2 &&judge_bt_points(ret.pt,s2->startpt,s2->endpt)==2)
						{
							ret.vals=1; //intersected (this is segment_t not line)
						}
						else
							ret.vals=0; //not intersected (this is segment_t not line)
					}
				}
				else if (s1->parab==0 && s2->parab!=0)
				{
					if((s2->startpt.x-s1->startpt.x)*(s2->endpt.x-s1->startpt.x)<0)
					{
						
						ret.pt.y=s2->parak*s1->startpt.x+s2->parap;
						ret.pt.x=s1->startpt.x;
						if((s1->startpt.y-ret.pt.y)*(s1->endpt.y-ret.pt.y)<0)
						{
							ret.vals=1;
						}
						else
							ret.vals=0;
					}
					else
					{
						ret.vals=0;
					}
				}
				else if (s1->parab!=0 && s2->parab==0)
				{
					if((s1->startpt.x-s2->startpt.x)*(s1->endpt.x-s2->startpt.x)<0)
					{
						
						ret.pt.y=s1->parak*s2->startpt.x+s1->parap;
						ret.pt.x=s2->startpt.x;
						if((s2->startpt.y-ret.pt.y)*(s2->endpt.y-ret.pt.y)<0)
						{
							ret.vals=1;
						}
						else
							ret.vals=0;
					}
					else
					{
						ret.vals=0;
					}
				}
				else
				{
					if(s1->parap==s2->parap || abs(s1->parap-s2->parap)<1e-6)
					{
						ret.vals==2;
						if (pt_pt_dist(s1->startpt,s2->startpt)<pt_pt_dist(s1->startpt,s2->endpt))
							ret.pt=s2->startpt;
						else
							ret.pt=s2->endpt;
					}
					else
					{
						ret.vals=0;
					}


				}

				return ret;
			}


			

			_numnpt offsetptrand(points_t refpt,points_t reftopt,double oset,double dlineangle)
			{
				double xr,yr,xt,yt;
				oset=abs(oset);
				xr=refpt.x;yr=refpt.y;
				xt=reftopt.x;yt=reftopt.y;
				double x = xr + (oset/pt_pt_dist(refpt,reftopt))*(xt-xr);
				double y = yr + (oset/pt_pt_dist(refpt,reftopt))*(yt-yr);
				vector<double> pol;
				pol.resize(3);
				pol=cart2pol(x-xr,y-yr,0);
				double theta,rho;
				//srand(time(NULL));
				srand(rand());
				theta=pol[0]+(rand()/double(RAND_MAX)-0.5)*(dlineangle/180)*PI;
				rho=pol[1]+rand()/double(RAND_MAX)*oset/2;
				vector<double> cart;cart.resize(2);
				cart=pol2cart(theta,rho);
				cart[0]=cart[0]+xr;
				cart[1]=cart[1]+yr;
				points_t ret;
				ret.x=cart[0];
				ret.y=cart[1];
				ret.z=reftopt.z;
				_numnpt reval;
				reval.pt=ret;
				reval.vals=rho;
				return reval;
			}

			points_t offsetpt(points_t refpt,points_t reftopt,double oset)
			{
				double xr,yr,xt,yt,x,y;
				xr=refpt.x;yr=refpt.y;
				xt=reftopt.x;yt=reftopt.y;
				x=xt+oset/pt_pt_dist(refpt,reftopt)*(xt-xr);
				y=yt+oset/pt_pt_dist(refpt,reftopt)*(yt-yr);
				points_t ret;
				ret.x=x;
				ret.y=y;
				ret.z=0;
				return ret;
			}


			points_t extend_direction(points_t refpt,points_t reftopt,double times)
			{
				points_t ret;
				segment_t seg;
				seg.startpt=refpt;
				seg.endpt=reftopt;
				seg.cal_para();
				if(reftopt.x!=refpt.x)
				{
					ret.x=refpt.x+(reftopt.x-refpt.x)*times;
					if(seg. paraa!=0 && seg. parab!=0)
					{
						ret.y=seg. parak*ret.x+seg. parap;
					}
					else if(seg.paraa==0 && seg.parab!=0)
					{
						ret.y=reftopt.y;
					}
					else
					{
						ret.y=(reftopt.y-refpt.y)*times+refpt.y;
					}
					ret.z=reftopt.z;
				}
				else
				{
					ret.x=reftopt.x;
					ret.y=(reftopt.y-refpt.y)*times+refpt.y;
					ret.z=reftopt.z;
				}

				return ret;
			}

			
			void vector2point(points_t& pt, vector<double>& vec)
			{
				if (vec.size()==3)
				{
					pt.x=vec[0];
					pt.y=vec[1];
					pt.z=vec[2];
				}
				else if (vec.size()==2)
				{
					pt.x=vec[0];
					pt.y=vec[1];
					pt.z = -999;
				}
			}

			void point2vector(vector<double>& vec, points_t& pt)
			{
				vec[0]=pt.x;
				vec[1]=pt.y;
				vec[2]=pt.z;
			}

			void calculate_buffer(segment_t* seg,segment_t* seg1,segment_t* seg2, double sunit)
			{
				if(seg->startpt.equal(seg->endpt))
				{
					
					seg1->set(seg->startpt,seg->endpt);
					seg2->set(seg->startpt,seg->endpt);
				}
				else
				{
					vector<double> pol,cart;
					double theta,rho;
					pol.resize(3);
					cart.resize(2);
					pol=cart2pol(seg->endpt.x-seg->startpt.x,seg->endpt.y-seg->startpt.y,0);
					theta=pol[0];
					rho=sunit;
					cart=pol2cart(theta+0.5*PI,rho);
					seg1->startpt.set(cart[0]+seg->startpt.x,cart[1]+seg->startpt.y,seg->startpt.z);
					cart=pol2cart(theta-0.5*PI,rho);
					seg2->startpt.set(cart[0]+seg->startpt.x,cart[1]+seg->startpt.y,seg->startpt.z);

					pol=cart2pol(seg->startpt.x-seg->endpt.x,seg->startpt.y-seg->endpt.y,0);
					theta=pol[0];
					rho=sunit;
					cart=pol2cart(theta-0.5*PI,rho);
					seg1->endpt.set(cart[0]+seg->endpt.x,cart[1]+seg->endpt.y,seg->endpt.z);
					cart=pol2cart(theta+0.5*PI,rho);
					seg2->endpt.set(cart[0]+seg->endpt.x,cart[1]+seg->endpt.y,seg->endpt.z);

					seg1->cal_para();seg2->cal_para();
				}
				seg1->object_id = seg->object_id;
				seg2->object_id = seg->object_id;
			}


			segment_struct_t::segment_struct_t()
			{
				object_id=-1;
				segment_id=-1;
				actual_segment = new segment_t();
				buffer_segment_left = new segment_t();
				buffer_segment_right = new segment_t();
			}

			segment_struct_t::segment_struct_t(int object_id, int segment_id, points_t start_point, points_t end_point)
			{
				this->object_id = object_id;
				this->segment_id = segment_id;
				actual_segment = new segment_t(object_id, start_point, end_point);
				buffer_segment_left = new segment_t();
				buffer_segment_right = new segment_t();	
				calculate_buffer(actual_segment,buffer_segment_left,buffer_segment_right);
			}

			void segment_struct_t::update(points_t start_point, points_t end_point)
			{
				actual_segment->set(start_point,end_point);
				calculate_buffer(actual_segment,buffer_segment_left,buffer_segment_right);
			}
			

			int segment_struct_t::get_hash_key()
			{
				return object_id*10000+segment_id;
			}

			double calculate_orientation(const points_t &prev, const points_t &cur)
			{
				return atan2(prev.y - cur.y , prev.x - cur.x );
				// if( fabs(cur.x - prev.x) < 1e-3 )
				// {
				// 	return prev.y>cur.y?PRX_HALF_PI:1.5*PRX_HALF_PI;
				// }
				// return (prev.y - cur.y)/(prev.x - cur.x);
			}
        }//namespace crowd

    }

}