#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <stdio.h>
#include <ctime>
#include <climits>

#include <helper/estd.h>

using namespace std;
using namespace estd;

#define IDRANGE 1000000
#define MAXAGENTS 4
#define MIN 0
#define MAX 999999

// ++++++++++++++++++ if func test ++++++++++++++++++++++++

size_t GetID(idpair Idp, bool bIsKf)
{
    if(bIsKf) return IDRANGE * Idp.second + Idp.first;
    else return IDRANGE * ( MAXAGENTS + Idp.second) + Idp.first;
}

idpair GetPair(size_t Id)
{
    size_t cid, id;
    cid = Id/IDRANGE;
    id = Id - cid * IDRANGE;
    if(cid >= MAXAGENTS) cid -= MAXAGENTS;
    return make_pair(id,cid);
}

size_t rn()
{
    return static_cast<size_t>(MIN + (rand() % (size_t)(MAX - MIN + 1)));
}

void show(size_t cid, bool bIsKf)
{
    idpair idp = make_pair(rn(),cid);
    size_t id = GetID(idp, bIsKf);
    idpair idpret = GetPair(id);
    cout << "Input pair: " << idp.first << "|" << idp.second << " --- GetId(idp): " << id << "--- GetPair(id):" << idpret.first << "|" << idpret.second << endl;
}

void show2(size_t sid, size_t cid, bool bIsKf)
{
    idpair idp = make_pair(sid,cid);
    size_t id = GetID(idp, bIsKf);
    idpair idpret = GetPair(id);
    string type;
    if(bIsKf) type = "KF";
    else type = "MP";
    cout << type << "--- Input pair: " << idp.first << "|" << idp.second << " --- GetId(idp): " << id << "--- GetPair(id):" << idpret.first << "|" << idpret.second << endl;
}

int main(int argc, char **argv) {

    cout << "UpdateStats Test" << endl;

    double x1=1.0,x2=2.3,x3=32.0,x4=17.23,x5=11.345;
    double mean=0.0,var=0.0,ssd=0.0;
    size_t n=0;

    var = ssd / static_cast<double>(n);
    cout << "mean|ssd|var|n: " << mean << "|" << ssd << "|" << var << "|" << n << endl;

    estd::UpdateStats(mean,ssd,n,x1);
    var = ssd / static_cast<double>(n);
    cout << "mean|ssd|var|n: " << mean << "|" << ssd << "|" << var << "|" << n << endl;

    estd::UpdateStats(mean,ssd,n,x2);
    var = ssd / static_cast<double>(n);
    cout << "mean|ssd|var|n: " << mean << "|" << ssd << "|" << var << "|" << n << endl;

    estd::UpdateStats(mean,ssd,n,x3);
    var = ssd / static_cast<double>(n);
    cout << "mean|ssd|var|n: " << mean << "|" << ssd << "|" << var << "|" << n << endl;

    estd::UpdateStats(mean,ssd,n,x4);
    var = ssd / static_cast<double>(n);
    cout << "mean|ssd|var|n: " << mean << "|" << ssd << "|" << var << "|" << n << endl;

    estd::UpdateStats(mean,ssd,n,x5);
    var = ssd / static_cast<double>(n);
    cout << "mean|ssd|var|n: " << mean << "|" << ssd << "|" << var << "|" << n << endl;


    cout << "CHAR_BIT" << CHAR_BIT << endl;
/*
    cout << " +++ size_t +++ " << endl;

    size_t uiTest = -1;
    cout << "uiTest: " << uiTest << endl;

    srand (time(NULL));

    cout << " +++ KeyFrames +++ " << endl;

    cout << "-Random-" << endl;
    show(0,true);
    show(0,true);
    show(0,true);
    show(1,true);
    show(1,true);
    show(1,true);
    show(2,true);
    show(2,true);
    show(2,true);
    show(3,true);
    show(3,true);
    show(3,true);
    cout << "-Chosen-" << endl;


    cout << " +++ MapPoints +++ " << endl;

    cout << "-Random-" << endl;
    show(0,false);
    show(0,false);
    show(0,false);
    show(1,false);
    show(1,false);
    show(1,false);
    show(2,false);
    show(2,false);
    show(2,false);
    show(3,false);
    show(3,false);
    show(3,false);

    cout << " +++ Mixed +++ " << endl;

    show2(0,0,true);
    show2(0,0,false);
    show2(0,1,true);
    show2(0,1,false);
    show2(0,2,true);
    show2(0,2,false);
    show2(0,3,true);
    show2(0,3,false);

    show2(11,0,true);
    show2(11,0,false);
    show2(11,1,true);
    show2(11,1,false);
    show2(11,2,true);
    show2(11,2,false);
    show2(11,3,true);
    show2(11,3,false);

    show2(1999,0,true);
    show2(1999,0,false);
    show2(1999,1,true);
    show2(1999,1,false);
    show2(1999,2,true);
    show2(1999,2,false);
    show2(1999,3,true);
    show2(1999,3,false);

    show2(923859,0,true);
    show2(923859,0,false);
    show2(923859,1,true);
    show2(923859,1,false);
    show2(923859,2,true);
    show2(923859,2,false);
    show2(923859,3,true);
    show2(923859,3,false);
*/
}

// ++++++++++++++++++ template test ++++++++++++++++++++++++

template <typename T, class C> class base : public boost::enable_shared_from_this< base<T,C> >
{
public:
    typedef typename C::user_t user_t;
    typedef boost::shared_ptr<user_t> uptr;
public:
    base(T t, uptr pU) : mT(t), mpU(pU) {}
    virtual string name(){return "base";}
    virtual T value(){return mT;}
    virtual void use(){mpU->setT(this->shared_from_this());}
private:
    T mT;
    uptr mpU;
};

template <typename T,class C> class derived : public base<T,C>//, public boost::enable_shared_from_this< derived<T,U> >
{
public:
    typedef typename C::user_t user_t;
    typedef boost::shared_ptr<user_t> uptr;
public:
    using base<T,C>::base;
    derived(T t, uptr pU, string name) : base<T,C>(t,pU), mName(name) {}
    virtual string name(){return mName;}
private:
    string mName;
    //...
};

template <class C> class user
{
public:
    typedef typename C::base_t base_t;
    typedef boost::shared_ptr<base_t> Tptr;
public:
    user(){};
    void setT(Tptr pT){mpT = pT;}
    void show(){cout << "User's mpT: name|value: " << mpT->name() << "|" << mpT->value() << endl;}
private:
    Tptr mpT;
};

struct combine
{
    typedef base<double,combine> base_t;
    typedef derived<double,combine> derived_t;
    typedef user<combine> user_t;
};

/*
 * // works
template <typename T, class U> class base : public boost::enable_shared_from_this< base<T,U> >
{
public:
    typedef boost::shared_ptr<U> uptr;
public:
    base(T t, uptr pU) : mT(t), mpU(pU) {}
    virtual string name(){return "base";}
    virtual T value(){return mT;}
    virtual void use(){mpU->setT(this->shared_from_this());}
private:
    T mT;
    uptr mpU;
};

template <typename T,class U> class derived : public base<T,U>//, public boost::enable_shared_from_this< derived<T,U> >
{
public:
    typedef boost::shared_ptr<U> uptr;
public:
    using base<T,U>::base;
    derived(T t, uptr pU, string name) : base<T,U>(t,pU), mName(name) {}
    virtual string name(){return mName;}
private:
    string mName;
    //...
};

class user
{
public:
    typedef boost::shared_ptr<base<double,user>> Tptr;
public:
    user(){};
    void setT(Tptr pT){mpT = pT;}
    void show(){cout << "User's mpT: name|value: " << mpT->name() << "|" << mpT->value() << endl;}
private:
    Tptr mpT;
};
*/

class tc
{
public:
    tc(int i){mX=i;}
    int getX() {return mX;}
protected:
    int mX;
};

template<class C> class mytempclass
{
public:
    mytempclass(int i) : mC(i) {}
    void showCX() {cout << mC.getX() << endl;}
private:
    C mC;
};

int main_old2(int argc, char **argv) {

    cout << "template class test" << endl;

    mytempclass<tc> mtp(7);
    mtp.showCX();


    cout << "template class and inheritance THIS test" << endl;

    boost::shared_ptr<user<combine>> pU(new user<combine>);
    boost::shared_ptr<base<double,combine>> pD(new derived<double,combine>(12.34,pU,"der0"));
    pD->use();
    pU->show();

    //--------------
//    boost::shared_ptr<combine> pC(new combine);
//    pC->base_t

//    boost::shared_ptr<user> pU(new user);
//    boost::shared_ptr<base<double,user>> pD(new derived<double,user>(12.34,pU,"der0"));
//    pD->use();
//    pU->show();

}




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef boost::shared_ptr<double> dptr;

class ptrtest
{
public:
    ptrtest(dptr pd) : mpd(pd) {}
    void ptrreset() {mpd.reset();}
    void ptrresetnew() {mpd.reset(new double);}
    void ptrreset(double d) {mpd.reset(new double(d));}
    void ptrsetnullptr() {mpd=nullptr;}
    void ptrset(double d) {*mpd=d;}
    void ptrsetnull() {*mpd=static_cast<double>(NULL);}
    bool isnull() {return(!mpd);}
    double value() {return *mpd;}
    int usecount() {return mpd.use_count();}
private:
    dptr mpd;
};

class myclass
{
public:
    myclass(size_t a, size_t b, double v) : ida(a),idb(b),val(v) {}
    void getvals(){std::cout << "ida|idb|val: " << ida << "|" << idb << "|" << val << endl;}
private:
    size_t ida;
    size_t idb;
    double val;
};

typedef boost::shared_ptr<myclass> cptr;

class classholder
{
public:
    classholder(cptr pClass) : mpClass(pClass) {}
    void GetMemberVals() {mpClass->getvals();}
    void GetMemberUseCount() {cout << "Usecount: " << mpClass.use_count() << endl;}
private:
    cptr mpClass;
};

typedef boost::shared_ptr<classholder> hptr;

class oldptrtest
{
public:
    oldptrtest(double* pd) : mpd(pd) {}
    void ptrdelete() {delete mpd;}
    void ptrsetnullptr() {mpd=nullptr;}
    void ptrset(double d) {*mpd=d;}
    bool isnull() {return(!mpd);}
private:
    double* mpd;
};

int main_old(int argc, char **argv) {

    cout << "started ptrtest" << endl;

    cout << "****** Color Test ******" << endl;

    cout << "\033[41;2;31m bold red text \033[0m" << endl;
    cout << "\033[1;33m bold yellow text \033[0m" << endl;

    ros::init(argc, argv, "mcptestnode");

    cout << "****** LIST TEST ******" << endl;

    list<cptr> lTest;
    list<cptr>::iterator lit;

    cptr pc0{new myclass(0,0,2.0)};
    cptr pc1{new myclass(1,1,4.5)};
    cptr pc2{new myclass(2,2,23.34)};
    cptr pc3{new myclass(3,3,1.25)};
    cptr pc4{new myclass(1,1,4.5)};
    cptr pcx{new myclass(2,2,23.34)};

    cout << "pc0: "; pc0->getvals();
    cout << "pc1: "; pc1->getvals();
    cout << "pc2: "; pc2->getvals();
    cout << "pc3: "; pc3->getvals();
    cout << "pc4: "; pc4->getvals();
    cout << "pcx: "; pcx->getvals();

    cout << "Push to list: 0 - 1 - 2 - 3 - 4 - 3" << endl;
    lTest.push_back(pc0);
    lTest.push_back(pc1);
    lTest.push_back(pc2);
    lTest.push_back(pc3);
    lTest.push_back(pc4);
    lTest.push_back(pc3);

    cout << "Find pc1 - expected: TRUE" << endl;
    lit = find(lTest.begin(),lTest.end(),pc1);
    if(lit!=lTest.end()) cout << "pc1 found" << endl;
    else cout << "pc1 not found" << endl;

    cout << "Erase pc1" << endl;
    lTest.erase(lit);

    cout << "Find again pc1 - expected: FALSE" << endl;
    lit = find(lTest.begin(),lTest.end(),pc1);
    if(lit!=lTest.end()) cout << "pc1 found" << endl;
    else cout << "pc1 not found" << endl;

    cout << "Find pcx - expected: FALSE" << endl;
    lit = find(lTest.begin(),lTest.end(),pcx);
    if(lit!=lTest.end()) cout << "pcx found" << endl;
    else cout << "pcx not found" << endl;

    cout << "Find pc3 - expected: TRUE" << endl;
    lit = find(lTest.begin(),lTest.end(),pc3);
    if(lit!=lTest.end()) cout << "pc3 found" << endl;
    else cout << "pc3 not found" << endl;

    cout << "Erase pc3" << endl;
    lTest.erase(lit);

    cout << "Find pc3 again - expected: TRUE" << endl;
    lit = find(lTest.begin(),lTest.end(),pc3);
    if(lit!=lTest.end()) cout << "pc3 found" << endl;
    else cout << "pc3 not found" << endl;

    cout << "****** POINTER TO CLASS TEST ******" << endl;

    cptr pc9{new myclass(1,2,34.5)};

    hptr ph0{new classholder(pc9)};
    hptr ph1{new classholder(pc9)};
    hptr ph2{new classholder(pc9)};

    cout << "All Set - Values:" << endl;

    cout << "ph0: "; ph0->GetMemberVals(); ph0->GetMemberUseCount();
    cout << "ph1: "; ph1->GetMemberVals(); ph1->GetMemberUseCount();
    cout << "ph2: "; ph2->GetMemberVals(); ph2->GetMemberUseCount();

    cout << "Change pc9: - Values: " << endl;

    *pc9 = myclass(6,7,8.9);

    cout << "ph0: "; ph0->GetMemberVals(); ph0->GetMemberUseCount();
    cout << "ph1: "; ph1->GetMemberVals(); ph1->GetMemberUseCount();
    cout << "ph2: "; ph2->GetMemberVals(); ph2->GetMemberUseCount();

    cout << "Change pc9 with operator= - Values: " << endl;

    cptr pc8{new myclass(9,9,99.9)};
    pc9 = pc8;

    cout << "ph0: "; ph0->GetMemberVals(); ph0->GetMemberUseCount();
    cout << "ph1: "; ph1->GetMemberVals(); ph1->GetMemberUseCount();
    cout << "ph2: "; ph2->GetMemberVals(); ph2->GetMemberUseCount();

    cout << "New Setting - Values: " << endl;

    cptr pc10{new myclass(10,10,10.10)};
    cptr pc11{new myclass(11,11,11.11)};

    hptr ph3{new classholder(pc10)};
    hptr ph4{new classholder(pc10)};
    hptr ph5{new classholder(pc10)};

    hptr ph6{new classholder(pc11)};
    hptr ph7{new classholder(pc11)};

    cout << "ph3: "; ph3->GetMemberVals(); ph3->GetMemberUseCount();
    cout << "ph4: "; ph4->GetMemberVals(); ph4->GetMemberUseCount();
    cout << "ph5: "; ph5->GetMemberVals(); ph5->GetMemberUseCount();
    cout << "ph6: "; ph6->GetMemberVals(); ph6->GetMemberUseCount();
    cout << "ph7: "; ph7->GetMemberVals(); ph7->GetMemberUseCount();

    cout << "Swap Test - Values: " << endl;

    cptr pc12{new myclass(12,12,12.12)};
    pc10.swap(pc12);
//    myclass* pctmp = pc10.get();
//    pctmp = new myclass(12,12,12.12);

    cout << "pc10: "; pc10->getvals(); cout << "usecount:" << pc10.use_count() << endl;
    cout << "pc11: "; pc11->getvals(); cout << "usecount:" << pc11.use_count() << endl;
    cout << "pc12: "; pc12->getvals(); cout << "usecount:" << pc12.use_count() << endl;

    cout << "ph3: "; ph3->GetMemberVals(); ph3->GetMemberUseCount();
    cout << "ph4: "; ph4->GetMemberVals(); ph4->GetMemberUseCount();
    cout << "ph5: "; ph5->GetMemberVals(); ph5->GetMemberUseCount();
    cout << "ph6: "; ph6->GetMemberVals(); ph6->GetMemberUseCount();
    cout << "ph7: "; ph7->GetMemberVals(); ph7->GetMemberUseCount();

    cout << "****** NEW TEST ******" << endl;

    dptr pd0 = nullptr;
    dptr pd1{new double(999.99)};
    dptr pd2{new double(888.88)};
    dptr pd3{new double(777.77)};

    ptrtest u1(pd0);
    ptrtest u2(pd0);
    ptrtest u3(pd0);
    cout << "u2.ptrreset(111.11)" << endl;
    u2.ptrreset(111.11);

    if(pd0) cout << "pd0: " << *pd0 << "; usecount: " << pd0.use_count() << endl; else cout << "pd0 is nullptr" << endl;
    if(!u1.isnull()) cout << "u1: " << u1.value() << "; usecount: " << u1.usecount() << endl; else cout << "u1 is nullptr" << endl;
    if(!u2.isnull()) cout << "u2: " << u2.value() << "; usecount: " << u2.usecount() << endl; else cout << "u2 is nullptr" << endl;
    if(!u3.isnull()) cout << "u3: " << u3.value() << "; usecount: " << u3.usecount() << endl; else cout << "u3 is nullptr" << endl;

    ptrtest v1(pd1);
    ptrtest v2(pd1);
    ptrtest v3(pd1);
    cout << "v2.ptrreset(222.22)" << endl;
    v2.ptrreset(222.22);

    if(pd1) cout << "pd1: " << *pd1 << "; usecount: " << pd1.use_count() << endl; else cout << "pd1 is nullptr" << endl;
    if(!v1.isnull()) cout << "v1: " << v1.value() << "; usecount: " << v1.usecount() << endl; else cout << "v1 is nullptr" << endl;
    if(!v2.isnull()) cout << "v2: " << v2.value() << "; usecount: " << v2.usecount() << endl; else cout << "v2 is nullptr" << endl;
    if(!v3.isnull()) cout << "v3: " << v3.value() << "; usecount: " << v3.usecount() << endl; else cout << "v3 is nullptr" << endl;

    ptrtest w1(pd2);
    ptrtest w2(pd2);
    ptrtest w3(pd2);
    cout << "w2.ptrset(333.33)" << endl;
    w2.ptrset(333.33);

    if(pd2) cout << "pd2: " << *pd2 << "; usecount: " << pd2.use_count() << endl; else cout << "pd2 is nullptr" << endl;
    if(!w1.isnull()) cout << "w1: " << w1.value() << "; usecount: " << w1.usecount() << endl; else cout << "w1 is nullptr" << endl;
    if(!w2.isnull()) cout << "w2: " << w2.value() << "; usecount: " << w2.usecount() << endl; else cout << "w2 is nullptr" << endl;
    if(!w3.isnull()) cout << "w3: " << w3.value() << "; usecount: " << w3.usecount() << endl; else cout << "w3 is nullptr" << endl;


//    cout << "****** OLD TEST ******" << endl;

//    dptr pd;
//    pd.reset(new double(123.45));

//    cout << "0 classes:" << endl;

//    cout << "use count:" << pd.use_count() << endl;
//    cout << "element:" << *pd << endl;

//    cout << "4 classes:" << endl;

//    ptrtest t1(pd);
//    cout << "is t1.mpd nullptr?" << t1.isnull() << endl;
//    ptrtest t2(pd);
//    cout << "is t2.mpd nullptr?" << t2.isnull() << endl;
//    ptrtest t3(pd);
//    cout << "is t3.mpd nullptr?" << t3.isnull() << endl;
//    ptrtest t4(pd);
//    cout << "is t4.mpd nullptr?" << t4.isnull() << endl;

//    cout << "use count:" << pd.use_count() << endl;
//    cout << "element:" << *pd << endl;

//    cout << "t1: ptrsetnullptr:" << endl;

//    t1.ptrsetnullptr();

//    cout << "use count:" << pd.use_count() << endl;
//    cout << "element:" << *pd << endl;
//    cout << "is t1.mpd nullptr?" << t1.isnull() << endl;

//    cout << "t2: ptrresetnew:" << endl;

//    t2.ptrresetnew();

//    cout << "use count:" << pd.use_count() << endl;
//    cout << "element:" << *pd << endl;
//    cout << "is t2.mpd nullptr?" << t2.isnull() << endl;

//    cout << "t3: ptrreset:" << endl;

//    t3.ptrreset();

//    cout << "use count:" << pd.use_count() << endl;
//    cout << "element:" << *pd << endl;
//    cout << "is t3.mpd nullptr?" << t3.isnull() << endl;

//    cout << "t4: ptrset(987.65):" << endl;

//    t4.ptrset(987.65);

//    cout << "use count:" << pd.use_count() << endl;
//    cout << "element:" << *pd << endl;

//    cout << "t4: ptrsetnull:" << endl;

//    t4.ptrsetnull();

//    cout << "use count:" << pd.use_count() << endl;
//    cout << "element:" << *pd << endl;
//    cout << "is t4.mpd nullptr?" << t4.isnull() << endl;

//    cout << "+++++++++++++++++++++++++++" << endl;
//    cout << "+++++++++++++++++++++++++++" << endl;
//    cout << "Old Ptr" << endl;

//    cout << "0 classes:" << endl;

//    double* opd = new double(123.45);

//    cout << "element:" << *opd << endl;

//    cout << "4 classes:" << endl;

//    oldptrtest to1(opd);
//    cout << "is to1.mpd nullptr?" << to1.isnull() << endl;
//    oldptrtest to2(opd);
//    cout << "is to2.mpd nullptr?" << to2.isnull() << endl;
//    oldptrtest to3(opd);
//    cout << "is to3.mpd nullptr?" << to3.isnull() << endl;
//    oldptrtest to4(opd);
//    cout << "is to4.mpd nullptr?" << to4.isnull() << endl;

//    cout << "element:" << *opd << endl;

//    cout << "t1: set(987.65):" << endl;

//    to1.ptrset(987.65);

//    cout << "element:" << *opd << endl;

//    cout << "t2: ptrsetnullptr:" << endl;

//    to2.ptrsetnullptr();

//    cout << "element:" << *opd << endl;
//    cout << "is to2.mpd nullptr?" << to2.isnull() << endl;

//    cout << "t3: ptrdelete:" << endl;

//    to3.ptrdelete();

//    cout << "element:" << *opd << endl;
//    cout << "is to3.mpd nullptr?" << to3.isnull() << endl;
}

//int main(int argc, char **argv) {

//    ros::init(argc, argv, "mcptestnode");



//    ros::spin();

//    /*
//    ros::Rate r(10);
//    while(ros::ok())
//    {
//        ros::spinOnce();
//        r.sleep();
//    }
//    */


//    return 0;
//}
