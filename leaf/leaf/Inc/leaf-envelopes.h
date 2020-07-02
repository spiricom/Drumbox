/*
 ==============================================================================
 
 leaf-envelopes.h
 Created: 20 Jan 2017 12:02:17pm
 Author:  Michael R Mulshine
 
 ==============================================================================
 */

#ifndef LEAF_ENVELOPES_H_INCLUDED
#define LEAF_ENVELOPES_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif
    
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    
#include "leaf-math.h"
#include "leaf-mempool.h"
#include "leaf-filters.h"
#include "leaf-delay.h"
#include "leaf-analysis.h"
#include "leaf-envelopes.h"
    
    /*!
     * @internal
     * Header.
     * @include basic-oscillators.h
     * @example basic-oscillators.c
     * An example.
     */

    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    
    /* Attack-Decay envelope */
    typedef struct _tEnvelope
    {
        tMempool mempool;
        
        const float *exp_buff;
        const float *inc_buff;
        uint32_t buff_size;
        
        float next;
        
        float attackInc, decayInc, rampInc;
        
        int inAttack, inDecay, inRamp;
        
        int loop;
        
        float gain, rampPeak;
        
        float attackPhase, decayPhase, rampPhase;
        
    } _tEnvelope;
    
    typedef _tEnvelope* tEnvelope;
    
    void    tEnvelope_init          (tEnvelope* const, float attack, float decay, int loop);
    void    tEnvelope_initToPool    (tEnvelope* const, float attack, float decay, int loop, tMempool* const);
    void    tEnvelope_free          (tEnvelope* const);
    
    float   tEnvelope_tick          (tEnvelope* const);
    void    tEnvelope_setAttack     (tEnvelope* const, float attack);
    void    tEnvelope_setDecay      (tEnvelope* const, float decay);
    void    tEnvelope_loop          (tEnvelope* const, int loop);
    void    tEnvelope_on            (tEnvelope* const, float velocity);
    
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    


    /* Exponential Smoother */
    typedef struct _tExpSmooth
    {
        tMempool mempool;
        float factor, oneminusfactor;
        float curr,dest;
    } _tExpSmooth;

    typedef _tExpSmooth* tExpSmooth;

    void    tExpSmooth_init         (tExpSmooth* const, float val, float factor);
    void    tExpSmooth_initToPool   (tExpSmooth* const, float val, float factor, tMempool* const);
    void    tExpSmooth_free         (tExpSmooth* const);

    float   tExpSmooth_tick         (tExpSmooth* const);
    float   tExpSmooth_sample       (tExpSmooth* const);
    void    tExpSmooth_setFactor    (tExpSmooth* const, float factor);
    void    tExpSmooth_setDest      (tExpSmooth* const, float dest);
    void    tExpSmooth_setVal       (tExpSmooth* const, float val);
    void    tExpSmooth_setValAndDest(tExpSmooth* const expsmooth, float val);

    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~


    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~


    /* ADSR */
        typedef struct _tADSR
        {
            tMempool mempool;
            
            const float *exp_buff;
            const float *inc_buff;
            uint32_t buff_size;

            float next;

            float attackInc, decayInc, releaseInc, rampInc;

            oBool inAttack, inDecay, inSustain, inRelease, inRamp;

            float sustain, gain, rampPeak, releasePeak;

            float attackPhase, decayPhase, releasePhase, rampPhase;

            float leakFactor;
        } _tADSR;

        typedef _tADSR* tADSR;

        void    tADSR_init    (tADSR* const adsrenv, float attack, float decay, float sustain, float release);
        void    tADSR_initToPool    (tADSR* const adsrenv, float attack, float decay, float sustain, float release, tMempool* const mp);
        void    tADSR_free          (tADSR* const);

        float   tADSR_tick          (tADSR* const);
        void    tADSR_setAttack     (tADSR* const, float attack);
        void    tADSR_setDecay      (tADSR* const, float decay);
        void    tADSR_setSustain    (tADSR* const, float sustain);
        void    tADSR_setRelease    (tADSR* const, float release);
        void    tADSR_setLeakFactor (tADSR* const, float leakFactor);
        void    tADSR_on            (tADSR* const, float velocity);
        void    tADSR_off           (tADSR* const);


    
    
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    


    /* ADSR2 */
     typedef struct _tADSR2
     {
         tMempool mempool;
         float sampleRateInMs;
         float attack;
         float decay;
         float release;
         float attackLambda;
         float decayLambda;
         float releaseLambda;
         float sustain;
         float leakGain;
         float leakFactor;
         float targetGainSquared;
         float factor;
         float oneMinusFactor;
         float gain;
         uint8_t attacking;
         uint8_t gate;
         float env;
         float envTarget;
     } _tADSR2;

     typedef _tADSR2* tADSR2;

     void    tADSR2_init          (tADSR2* const, float attack, float decay, float sustain, float release);
     void    tADSR2_initToPool    (tADSR2* const, float attack, float decay, float sustain, float release, tMempool* const);
     void    tADSR2_free          (tADSR2* const);

     float   tADSR2_tick          (tADSR2* const);
     void    tADSR2_setAttack     (tADSR2* const, float attack);
     void    tADSR2_setDecay      (tADSR2* const, float decay);
     void    tADSR2_setSustain    (tADSR2* const, float sustain);
     void    tADSR2_setRelease    (tADSR2* const, float release);
     void    tADSR2_setLeakFactor (tADSR2* const, float leakFactor);
     void    tADSR2_on            (tADSR2* const, float velocity);
     void    tADSR2_off           (tADSR2* const);


     // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

     // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
     enum envState {
         env_idle = 0,
         env_attack,
         env_decay,
         env_sustain,
         env_release,
         env_ramp
     };

     /* ADSR3 */
     typedef struct _tADSR3
     {
         tMempool mempool;
         float sampleRateInMs;
         int state;
         float output;
         float attackRate;
         float decayRate;
         float releaseRate;
         float attackCoef;
         float decayCoef;
         float releaseCoef;
         float sustainLevel;
         float targetRatioA;
         float targetRatioDR;
         float attackBase;
         float decayBase;
         float releaseBase;
         float leakFactor;
         float targetGainSquared;
         float factor;
         float oneMinusFactor;
         float gain;

     } _tADSR3;

     typedef _tADSR3* tADSR3;

     void    tADSR3_init          (tADSR3* const, float attack, float decay, float sustain, float release);
     void    tADSR3_initToPool    (tADSR3* const, float attack, float decay, float sustain, float release, tMempool* const);
     void    tADSR3_free          (tADSR3* const);

     float   tADSR3_tick          (tADSR3* const);
     void    tADSR3_setAttack     (tADSR3* const, float attack);
     void    tADSR3_setDecay      (tADSR3* const, float decay);
     void    tADSR3_setSustain    (tADSR3* const, float sustain);
     void    tADSR3_setRelease    (tADSR3* const, float release);
     void    tADSR3_setLeakFactor (tADSR3* const, float leakFactor);
     void    tADSR3_on            (tADSR3* const, float velocity);
     void    tADSR3_off           (tADSR3* const);

    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    
    //ADSR4

     
     typedef struct _tADSR4
         {
             tMempool mempool;
             const float *exp_buff;
             uint32_t buff_size;
             uint32_t buff_sizeMinusOne;
             float bufferSizeDividedBySampleRateInMs;
             float next;

             float attackInc, decayInc, releaseInc, rampInc;

             uint32_t whichStage;

             float sustain, gain, rampPeak, releasePeak;

             float attackPhase, decayPhase, releasePhase, rampPhase;

             float leakFactor;
         } _tADSR4;

         typedef _tADSR4* tADSR4;

         void    tADSR4_init          (tADSR4* const, float attack, float decay, float sustain, float release, float* expBuffer, int bufferSize);
         void    tADSR4_initToPool    (tADSR4* const, float attack, float decay, float sustain, float release, float* expBuffer, int bufferSize, tMempool* const);
         void    tADSR4_free          (tADSR4* const);

         float   tADSR4_tick          (tADSR4* const);
         float   tADSR4_tickNoInterp  (tADSR4* const adsrenv);
         void    tADSR4_setAttack     (tADSR4* const, float attack);
         void    tADSR4_setDecay      (tADSR4* const, float decay);
         void    tADSR4_setSustain    (tADSR4* const, float sustain);
         void    tADSR4_setRelease    (tADSR4* const, float release);
         void    tADSR4_setLeakFactor (tADSR4* const, float leakFactor);
         void    tADSR4_on            (tADSR4* const, float velocity);
         void    tADSR4_off           (tADSR4* const);
     // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

    /* Ramp */
    typedef struct _tRamp
    {
        tMempool mempool;
        float inc;
        float inv_sr_ms;
        float minimum_time;
        float curr,dest;
        float time;
        float factor;
        int samples_per_tick;
    } _tRamp;
    
    typedef _tRamp* tRamp;
    
    void    tRamp_init          (tRamp* const, float time, int samplesPerTick);
    void    tRamp_initToPool    (tRamp* const, float time, int samplesPerTick, tMempool* const);
    void    tRamp_free          (tRamp* const);
    
    float   tRamp_tick          (tRamp* const);
    float   tRamp_sample        (tRamp* const);
    void    tRamp_setTime       (tRamp* const, float time);
    void    tRamp_setDest       (tRamp* const, float dest);
    void    tRamp_setVal        (tRamp* const, float val);

    ///=============
    /* Ramp Updown*/
    typedef struct _tRampUpDown
    {
        tMempool mempool;
        float upInc;
        float downInc;
        float inv_sr_ms;
        float minimum_time;
        float curr,dest;
        float upTime;
        float downTime;
        int samples_per_tick;
    } _tRampUpDown;

    typedef _tRampUpDown* tRampUpDown;

    void    tRampUpDown_init          (tRampUpDown* const, float upTime, float downTime, int samplesPerTick);
    void    tRampUpDown_initToPool    (tRampUpDown* const, float upTime, float downTime, int samplesPerTick, tMempool* const);
    void    tRampUpDown_free          (tRampUpDown* const);
    
    float   tRampUpDown_tick          (tRampUpDown* const);
    float   tRampUpDown_sample        (tRampUpDown* const);
    void    tRampUpDown_setUpTime       (tRampUpDown* const, float upTime);
    void    tRampUpDown_setDownTime       (tRampUpDown* const, float downTime);
    void    tRampUpDown_setDest       (tRampUpDown* const, float dest);
    void    tRampUpDown_setVal        (tRampUpDown* const, float val);


    ///=========================
    //Slide (based on Max/MSP's slide~)
	typedef struct _tSlide
    {
        tMempool mempool;
		float prevOut;
		float currentOut;
		float prevIn;
		float invUpSlide;
		float invDownSlide;
		float dest;
	} _tSlide;

	typedef _tSlide* tSlide;

	void    tSlide_init          (tSlide* const, float upSlide, float downSlide);
	void    tSlide_initToPool    (tSlide* const, float upSlide, float downSlide, tMempool* const);
	void    tSlide_free          (tSlide* const);

	void 	tSlide_setUpSlide	(tSlide* const sl, float upSlide);

	void 	tSlide_setDownSlide	(tSlide* const sl, float downSlide);


	void 	tSlide_setDest		(tSlide* const sl, float dest);


	float 	tSlide_tickNoInput	(tSlide* const sl);

	float   tSlide_tick         (tSlide* const, float in);

#ifdef __cplusplus
}
#endif

#endif  // LEAF_ENVELOPES_H_INCLUDED



