/* t. schneider 7.10.07
 * tes@mit.edu
 * mit / whoi joint program in ocean engineering
 * use of this software outside of the schmidt laboratory at mit requires permission of the author.
 *
 * this is a special purpose script. it is intended to correct data values from the may 2007 project-minus
 * cruise during which auv unicorn's towed array was incorrectly connected and the polarity of the samples flipped.
 * this script replaces the resulting incorrect data sent to moos (BTR_DATA entries in the corresponding *.alog files
 * produced by pLogger) by the PC 104 stack controlling the array. The values replaced are drawn from *.btr files
 * which are corrected data samples taken from the raw data stored on the PC 104 stack. The correspondence between
 * the two data sets (*.alog generated by moos and *.btr generated from corrected raw data) is made by comparing the
 * timestamps in the *.alog file (PAEL_TIME, generated by pAEL) to those in the raw data file (*.btr).
 *
 * this script takes two command line inputs:
 *      btr_fix input_file0.btr input_directory_with_alog_files
 * 
 * input_file0.btr must contain the corrected BTR_DATA points in the following format
 *      unix_timestamp:{BTR_DATA}
 *
 * input_directory_with_alog_files is a directory with subdirectories that must contain one or more *.alog files that
 * must contain moos variables amongst which are PAEL_TIME and BTR_DATA. the format is defined by the *.alog
 * file format in the moos system (see http://www.robots.ox.ac.uk/%7Epnewman/TheMOOS/index.html for detatils)
 * 
 * for each *.alog file found, an outputfile.alog is produced with the file format: {original_file_name}_btr_fixed.alog.
 * it is an exact replica of the input *.alog except that all BTR_DATA values are replaced
 * by those contained in input_file0.btr where the PAEL_TIME in input_file1.alog corresponds to the unix_timestamp
 * of input_file0.btr.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <dirent.h>


#define MAXLINE 10000   /* maximum input line size from file */
#define TIMESTAMPWIDTH  20      /* width of the timestamp in chars found in both in files */

int getline(FILE *, char [], int);
int addblanks(FILE *, int);
int fix_file(FILE *, FILE *, FILE *);
static int one (const struct dirent *);
int parse_file_name(char *, char *, char *);
int dir_search(char [], char *);

int main(int argc, char *argv[])
{       
    if (argc < 2)
    {   
        printf("btr_fix: missing arguments. syntax is 'btr_fix input_file0.btr input_directory_with_alog_files'.\n");
        return 1;
    }
        
    char direct_in[1000];
        
    if (argc < 3)
    {
        strcpy(direct_in, ".");
    }
    else
    {
        strcpy(direct_in, argv[2]);
    }

    dir_search(direct_in, argv[1]);

    return 0;
}

int dir_search(char directory[], char * btr_file)
{
        
    /* declare directory related vars */
    DIR *din;
    struct dirent **ein;
        
    printf("btr_fix: entering directory: '%s'\n", directory);
    int n;
    n = scandir (directory, &ein, one, alphasort);
    if (n >= 0)
    {
        int i;
        for (i = 0; i < n; ++i)
        {
            if ((ein[i]->d_type) == DT_DIR)
            {                           
                /* enter directory if it is not . or .. */
                if (strcmp(ein[i]->d_name, ".") && strcmp(ein[i]->d_name,".."))
                {
                    char direct_in[1000];
                    strcpy(direct_in, directory);
                    strcat(strcat(direct_in, "/"), ein[i]->d_name);
                                        
                    dir_search(direct_in, btr_file);
                }
            }
            /* check for regular file type */
            else if ((ein[i]->d_type) == DT_REG)
            {
                char name[256], extension[100];
                parse_file_name(ein[i]->d_name, name, extension);

                /* if it is an alog file, process it */
                if(!strcmp(extension, "alog"))
                {
                    printf("btr_fix: processing file: %s.%s\n", name, extension);
                                                
                    /* open the necessary files and check for success */
                    char fin1_full_name[2000], fout_full_name[2000];
                                                
                    /* create a string containing the full path to the input alog file */
                    strcpy(fin1_full_name, directory);
                    strcat(strcat(fin1_full_name, "/"), ein[i]->d_name);
                                                
                    /* create a string with the full path to the output new alog file */
                    strcpy(fout_full_name, directory);
                    strcat(strcat(strcat(strcat(fout_full_name,"/"),name),".btr_fixed."),extension);
                                                
                                                
                    /* file pointers */
                    FILE        *fin0; /* btr file */
                    FILE        *fin1; /* currently open alog */
                    FILE        *fout; /* currently open output file */
        
                    if ((fin0 = fopen(btr_file, "r")) == NULL)
                    {
                        printf("btr_fix: can't open %s.\n", btr_file);
                        return 1;
                    }
                    else if ((fin1 = fopen(fin1_full_name, "r")) == NULL)
                    {
                        printf("btr_fix: can't open %s.\n", fin1_full_name);
                        return 1;
                    }           
                    else if ((fout = fopen(fout_full_name, "w")) == NULL)
                    {
                        printf("btr_fix: can't open %s.\n", fout_full_name);
                        return 1;
                    }
                    /* all files opened ok */   
                        
                    fix_file(fin0, fin1, fout);
                        
                    printf("btr_fix: generated file: %s.btr_fixed.%s\n", name, extension);
                                                
                    /* close the files we used */
                    fclose(fin0);
                    fclose(fin1);
                    fclose(fout);
                }
                        
            }
                                
        }
        printf("btr_fix: leaving directory: '%s'\n", directory);                
        return 0;
    }
    else
    {
        printf("btr_fix: can't open the directory: '%s'", directory);
        return 1;
    }
        
    return 0;
}

        
/* given three files, fin0 = *.btr file, fin1 = *.alog file, fout = new *.alog file,
 * fix_file performs the switch of the BTR values described in the header comment of this program
 */


int fix_file(FILE *fin0, FILE *fin1, FILE *fout)
{       

    char line0[MAXLINE], line1[MAXLINE]; /* store the lines */
    int len0, len1; /* length of the lines from two input files */
        
    /* current values of timestamps are ints, where actual timestamp value is *_int.*_frac */
    int cur_PAEL_TIME_int = 0, cur_PAEL_TIME_frac = 0, cur_btr_time_int = 0, cur_btr_time_frac = 0, last_btr_time_int = 0, last_btr_time_frac = 0; 

    /* while reading non zero lines from *.alog, keep going */ 
    while ((len1 = getline(fin1, line1, MAXLINE)) > 0)
    {   
        /* parse the alog line into parts - timecode | variablename | device | value */                 
        char alog_code[100], alog_varname[100], alog_device[100], alog_value[MAXLINE - 300], cur_btr_value[MAXLINE-300], last_btr_value[MAXLINE-300];
        sscanf(line1, "%s %s %s %s", alog_code, alog_varname, alog_device, alog_value);

        /* if it is a PAEL_TIME line, store new PAEL_TIME value */
        if (!strcmp(alog_varname, "PAEL_TIME"))
        {
            sscanf(alog_value, "%u.%u", &cur_PAEL_TIME_int, &cur_PAEL_TIME_frac);
        }
                
        /* else if it is a BTR_DATA line, look for replacement in *.btr and make it if found */
        if (!strcmp(alog_varname, "BTR_DATA"))
        {
            //printf("btr_fix: Found BTR_DATA. Corresponds to PAEL_TIME: %u.%u\n", cur_PAEL_TIME_int, cur_PAEL_TIME_frac);
                        
            int last_diff, cur_diff; /* variables to hold difference between PAEL_TIME and btr_time */
                        
            /* keep looping through *.btr file until cur_btr_time_* is greater than cur_PAEL_TIME_*/
            /* subtract second portion (*_int) to prevent overflow, if they are within 100 seconds of each other
               /* do comparison by effectively multiplying the fractional timestamp by 1000000 to make it an int */
            if (abs(last_btr_time_int - cur_PAEL_TIME_int) < 100)
                last_diff = ((last_btr_time_int - cur_PAEL_TIME_int) * 1000000 + (last_btr_time_frac - cur_PAEL_TIME_frac));
            else        
                last_diff = (last_btr_time_int - cur_PAEL_TIME_int);
            if (abs(cur_btr_time_int - cur_PAEL_TIME_int) < 100)
                cur_diff = ((cur_btr_time_int - cur_PAEL_TIME_int) * 1000000 + (cur_btr_time_frac - cur_PAEL_TIME_frac));
            else
                cur_diff = (cur_btr_time_int - cur_PAEL_TIME_int);                              
            //printf("btr_fix: Computing diffs. last_diff: %e. cur_diff: %e\n", last_diff, cur_diff);
                        
            /* eof flag for BTR file. 1 = EOF reached */
            char btr_EOF = 0;
                        
            while((cur_btr_time_int == 0) || (cur_diff < 0))
            {                           
                if((len0 = getline(fin0, line0, MAXLINE)) > 0) /* if not reached end of btr, keep looking */
                {
                    last_btr_time_int = cur_btr_time_int;
                    last_btr_time_frac = cur_btr_time_frac;
                                                                                        
                    strcpy(last_btr_value, cur_btr_value);
                    sscanf(line0, "%u.%u:%s", &cur_btr_time_int, &cur_btr_time_frac, cur_btr_value);
                                        
                    // printf("btr_fix: Found new btr_time. Last: %u.%u Current: %u.%u\n", last_btr_time_int, last_btr_time_frac, cur_btr_time_int, cur_btr_time_frac);
                                        
                    /* recompute difference between *.btr file timestamp and cur_PAEL_TIME */
                    if (abs(last_btr_time_int - cur_PAEL_TIME_int) < 100)
                        last_diff = ((last_btr_time_int - cur_PAEL_TIME_int) * 1000000 + (last_btr_time_frac - cur_PAEL_TIME_frac));
                    else        
                        last_diff = (last_btr_time_int - cur_PAEL_TIME_int);
                    if (abs(cur_btr_time_int - cur_PAEL_TIME_int) < 100)
                        cur_diff = ((cur_btr_time_int - cur_PAEL_TIME_int) * 1000000 + (cur_btr_time_frac - cur_PAEL_TIME_frac));
                    else
                        cur_diff = (cur_btr_time_int - cur_PAEL_TIME_int);                                              
                    //printf("btr_fix: Computing diffs. last_diff: %e. cur_diff: %e\n", last_diff, cur_diff);
                }
                else /* no more *.btr file */
                {
                    btr_EOF = 1;
                    break;
                }
            }
                        
            /* pick closer of two timestamp values, ensuring *.btr value and PAEL_TIME are within 5 seconds of each */
            if((abs(cur_diff) < abs(last_diff)) && (abs(cur_diff) < 5 * 1000000) && (last_btr_time_int != 0) && (abs(cur_btr_time_int - cur_PAEL_TIME_int) < 100))
            {
                /* use cur_diff */
                strcpy(alog_value, cur_btr_value);
                printf("btr_fix: BTR_VALUE replacement found:\n\t%u.%u -- PAEL_TIME\n\t%u.%u -- *.btr time\n\t%f -- difference\n", cur_PAEL_TIME_int, cur_PAEL_TIME_frac, cur_btr_time_int, cur_btr_time_frac, ((double)cur_diff / 1000000));
            }
            else if ((abs(last_diff) < 5 * 1000000) && (last_btr_time_int != 0) && (abs(last_btr_time_int - cur_PAEL_TIME_int) < 100))
            {
                /* use last_diff */
                strcpy(alog_value, last_btr_value);
                printf("btr_fix: BTR_VALUE replacement found:\n\t%u.%u -- PAEL_TIME\n\t%u.%u -- *.btr time\n\t%f -- difference\n", cur_PAEL_TIME_int, cur_PAEL_TIME_frac, last_btr_time_int, last_btr_time_frac, ((double)last_diff / 1000000));
            }
            else if(btr_EOF)
            {
                /* end of file reached for *.btr, use NaN for BTR value */
                strcpy(alog_value, "NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN");
                printf("btr_fix: BTR_VALUE replacement *not* found. Cause: *.btr reached EOF.\n\t%u.%u -- PAEL_TIME\n\t%u.%u -- current *.btr time\n\t%u.%u -- last *.btr time\n", cur_PAEL_TIME_int, cur_PAEL_TIME_frac, cur_btr_time_int, cur_btr_time_frac, last_btr_time_int, last_btr_time_frac);
            }
            else
            {
                /* other failure, use NaN for BTR values */
                strcpy(alog_value, "NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN");
                printf("btr_fix: BTR_VALUE replacement *not* found. Cause: no *.btr timestamp in range.\n\t%u.%u -- PAEL_TIME\n\t%u.%u -- current *.btr time\n\t%u.%u -- last *.btr time\n", cur_PAEL_TIME_int, cur_PAEL_TIME_frac, cur_btr_time_int, cur_btr_time_frac, last_btr_time_int, last_btr_time_frac);
            }
                        
            /* rebuild data line. in alog file, character spacing is such (including non spaces)
             * | 16 chars | 21 chars | 11 chars | 12 chars |
             * if a value overextends its field, a newline is inserted */        
            int field_len = strlen(alog_code); /* length of non whites for each field */
            int blanks = 16 - field_len;        /* number of blanks to insert (spaces) */
            fprintf(fout, "%s", alog_code);
            addblanks(fout, blanks);
                        
            field_len = strlen(alog_varname);
            blanks = 21 - field_len;
            fprintf(fout, "%s", alog_varname);
            addblanks(fout, blanks);    
                        
            field_len = strlen(alog_device);
            blanks = 11 - field_len;
            fprintf(fout, "%s", alog_device);
            addblanks(fout, blanks);    
                        
            field_len = strlen(alog_value);
            blanks = 12 - field_len;
            fprintf(fout, "%s", alog_value);
            addblanks(fout, blanks);
                                
            putc('\n',fout);
                        
                        
        }
        else    /* output the line unmodified to the output file */
        {
            fprintf(fout, "%s", line1);
        }
                
    }
    return 0;
}


/* reads a line from the input source and returns the length
 * modified from function from Kernighan *The C Programming Language, 2nd Edition*
 */
int getline(FILE *fp, char s[], int lim)
{
    int c, i;
        
    for(i=0; i<lim-1 && (c=getc(fp))!=EOF && c!='\n'; ++i)
        s[i] = c;
    if (c == '\n')
    {
        s[i] = c;
        ++i;
    }
    s[i] = '\0';
    return i;
}

/* adds blanks to FILE *fout based on number in int blanks
 * if blanks is nonpositive, outputs a single space instead */
int addblanks(FILE *fout, int blanks)
{       
    if (blanks < 0)
    {   
        putc(' ',fout);
    }   
    else
    {
        for(blanks; blanks > 0; blanks--)
        {
            putc(' ',fout);
        }
    }
}

/* picks a file name string of form {name}.{extension} into its parts */
int parse_file_name(char *whole, char *name, char *ext)
{
    int i = 0, per_flag = 0, per_loc;
        
    do
    {   
        if ((whole[i] == '.') && (per_flag == 0))
        {       
            per_flag = 1;
            per_loc = i;
            name[i] = '\0';
        } 
        else if (!per_flag)
        {
            name[i] = whole[i];
        }
        else
        {
            ext[i-per_loc-1] = whole[i];
        }
        i++;    
    } while (whole[i-1] != '\0');
}


/* for use in reading all directories */
static int one (const struct dirent *unused)
{
    return 1;
}

